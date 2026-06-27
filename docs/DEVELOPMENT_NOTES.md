# VeriFlatten — Project Development Notes

## 架构说明

```
VeriFlatten/
├── src/main.py         # 主处理程序: Verilator JSON → 扁平化 Verilog
├── install_tools.sh    # 安装 oss-cad-suite + sv2v
├── oss-cad-suite/      # Verilator, Yosys, SymbiYosys 等工具
├── bench/
│   └── verification-benchmarks/
│       ├── hackatdac18/  # ✅ 完成 (combined_flat.sv 方式)
│       ├── hackatdac19/  # ❌ 阻塞 (顶层接口端口)
│       ├── hackatdac21/  # ✅ 完成 (37个剩余lint错误)
│       └── or1200/       # ❌ 未开始
├── regress/
│   └── eq_check/run.py   # 7个基准的回归+等价性检查
└── README                # 项目概述
```

## 关键发现

### Verilator JSON 结构
- 扁平化后的 JSON (`V{top}_990_final.tree.json`) 是主要处理对象
- 模块存储在 `modulesp` 列表中，顶层为 `$root`
- 每个模块内部的组织: `scopep → varsp`, `blocksp`, `stmtsp`
- 连续赋值被表示为 `keyword: "cont_assign"` 的 `ALWAYS` 块
- CONCAT 的操作数在 `lhsp` 中（不是 `exprsp`）

### 预处理器 token 限制
- Verilator 限制 ~40000 tokens/行
- `assign` 在模块级别是一个预处理器行，`always @(*) begin ... end` 内每个语句独立
- sv2v 会将单语句 always 块压缩为一行，再次触发限制
- **解决方案**: 拆分为临时 wire + assign 语句，每行保持小规模

### sv2v 行为
- 单语句 always 块 → 单行: `always @(*) stmt;`
- 多语句 always 块 → 保持多行格式
- `/* sv2v_keep */` 和 `if(1) begin end` 不能阻止压缩
- 唯一可靠方法：保持至少 2 个真实赋值语句

### CONCAT 分割策略
- 含 CASE 的 ASSIGNW 路径：`handle_assignw` 中的 `rhs_contains_case` 分支
- 不含 CASE 的路径：`handle_always` (cont_assign) → `handle_assignw` → `else` 分支
- CONCAT 分割应在 `depth=0` 的逗号处（即 `{}` 内部的顶层逗号）
- `depth=1` 逗号分割后块不平衡（缺少关闭括号），因为括号跨块

### hackatdac21 处理总结
- z_out 问题修复后最长行从 486K → 112K 字符（11K tokens，低于 40K 限制）
- 剩余 37 个 lint 错误来自 Verilator 基础限制，非 main.py 问题
- 所有 7 个回归测试通过
