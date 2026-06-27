# src/main.py — 关键代码路径说明

## 文件结构

- **~3500 行**, 核心类 `VerilogGenerator`
- 处理 Verilator AST JSON → 扁平化 Verilog
- 支持 `--file`（单文件）和 `--filelist`（文件列表）两种模式

## 核心处理流程

```
Verilator JSON (V{top}_990_final.tree.json)
    ↓
main(): 加载 JSON → 创建 VerilogGenerator
    ↓
VerilogGenerator.__init__(): 预处理模块层级结构
    │
    ├── collect_module_signals():  收集所有模块的信号声明
    ├── collect_always_blocks():   收集所有 always 块
    ├── collect_instances():       收集模块例化
    └── collect_assignments():     收集连续赋值 (ASSIGNW + CONT_ASSIGN)
    │
    ├── process_signals():         生成信号声明 (reg/wire/logic)
    ├── process_instances():       生成模块例化
    ├── process_always_blocks():   处理所有 always 块
    └── process_assignments():     处理所有 assign 语句
    │
    ↓
    process_sel_temp():            插入临时选择变量
    process_resultp():             插入表达式副作用语句
    insert_sel_temp_statements():  最终调整临时变量位置
    ↓
sv2v (可选) → 输出 Verilog 文件
```

## 关键函数

### handle_always() (第 1254 行)
- 处理 ALWAYS 节点
- `keyword == "cont_assign"` 的特殊分支用于连续赋值
  - 检查是否包含控制流 (CASE/LOOP/BEGIN)
  - 有控制流 → `always @(*) begin ... end`
  - 无控制流 → `assign` 语句（长行时包裹在 always 中）

### handle_assignw() (第 910 行)
- 处理 ASSIGNW 节点（wire 赋值）
- 先通过 `)[]` 循环提取位选择表达式为 temp wires
- 然后检查 RHS 是否包含 CASE 节点
  - CASE 路径: 拆分为 `always @(*)` 或递归 CONCAT 分解
  - 非 CASE 路径: 使用 `assign` 语句，长行时自动分割

### handle_concat() (第 2326 行)
- 处理 CONCAT 节点（`{a, b, c}`）
- 简单拼接左右操作数

### handle_expression() (第 2100 行附近)
- 表达式分发器，根据 `node["type"]` 调用对应的 handle_* 方法

### insert_sel_temp_statements() (第 2561 行)
- 在正确位置插入 `___sel_temp_N` 的声明和赋值
- 按照变量��号从大到小排序插入

## 重要的配置变量

### CONVERT_ASSIGN_TO_ALWAYS (第 17 行)
- `False`: `assign` 语句格式（默认）
- `True`: `always @(*)` 格式（用于等价性检查）

### SEPARATE_ASSIGN_FOR_EQ_CHECK
- 控制是否为等价性检查生成特殊格式

## 已修复 Bug 的位置

### Bug 1: INSIDERANGE in CASEITEM
- 位置: `handle_case()` 中 condsp 处理
- 检测 `insiderange_type`，若是则将 `[left:right]` 展开为逗号分隔常量

### Bug 2: CONCAT 超长行 (z_out)
- 位置: `handle_assignw()` 第 1074 行附近的 `len(rhs) > 200000` 分支
- 修复: 用递归 `extract_concat_items()` 函数替代原来的 `max_chunk` 分割

### Bug 3: sel_temp_dict TypeError
- 位置: `insert_sel_temp_statements()` 第 2586 行
- 修复: 添加 `isinstance(info, dict)` 守卫
