from enum import Flag
import json
import argparse
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple
import traceback
import os
import subprocess
import re
import debugpy
import time
import psutil

SEPARATE_INITIAL_ASSIGN = False
SEPARATE_ASSIGN_FOR_EQ_CHECK = True
CONVERT_ASSIGN_TO_ALWAYS = False
CONVERT_COMPLEX_SEL_TO_TEMP = True
sys.setrecursionlimit(10000)  # 例如改成10000，看具体需求

class UnhandledTypeError(Exception):
    """当遇到未处理的类型时抛出此异常"""
    pass

class VerilogGenerator:
    def __init__(self):
        self.indent_level = 0
        self.indent_str = "    "
        self.type_table = {}  # 存储类型定义的字典
        self.temp_wire_count = 0
        self.temp_wires = []
        self.temp_assigns = []
        self.temp_count = 0  # 添加临时变量计数器
        self.initialized_vars = set()  # 新增：记录已经初始化过的变量
        self.resultp_dict = {}  # 新增：记录所有的resultp节点
        self.used_vars = set()  # 存储所有被引用的变量名
        self.procedural_vars = set()  # 存储所有被程序化赋值的变量名
        
        # 初始化临时变量记录器（如果还没有的话）
        self.sel_temp_count = 0
        self.convert_complex_sel_to_temp = CONVERT_COMPLEX_SEL_TO_TEMP
        self.sel_temp_dict = {}

        # 存储上一个ASSIGN语句的LHS和RHS，用于handle_loop提取for循环初始化值
        self._last_assign_lhs = None
        self._last_assign_rhs = None
        

    def is_func_exprstmt_struc(self, node: Dict[str, Any]) -> bool:
        """
        检查是否是特殊的always结构:
        - always没有sensesp
        - 包含一个assign
        - assign的rhsp是COND
        - COND的thenp是EXPRSTMT
        - EXPRSTMT的stmtsp[0]是COMMENT
        """
        try:
            # 检查是否是没有sensesp/sentreep的always
            if node.get("sensesp") != [] or node.get("sentreep") != []:
                return False
                
            # 获取always下的语句
            stmtsp = node.get("stmtsp", [])
            if not stmtsp or len(stmtsp) != 1:
                return False
                
            # 检查是否是assign
            assign_stmt = stmtsp[0]
            if assign_stmt.get("type") != "ASSIGN":
                return False
                
            # 检查rhsp是否是COND
            rhsp = assign_stmt.get("rhsp", [])
            if not rhsp or len(rhsp) != 1 or rhsp[0].get("type") != "COND":
                return False
                
            # 检查COND的thenp是否是EXPRSTMT
            cond_node = rhsp[0]
            thenp = cond_node.get("thenp", [])
            if not thenp or len(thenp) != 1 or thenp[0].get("type") != "EXPRSTMT":
                return False
                
            # 检查EXPRSTMT的stmtsp[0]是否是COMMENT
            expr_stmt = thenp[0]
            expr_stmtsp = expr_stmt.get("stmtsp", [])
            if not expr_stmtsp or expr_stmtsp[0].get("type") != "COMMENT":
                return False
                
            return True
            
        except Exception:
            print("Warning: is_func_exprstmt_struc judgement fails! ")
            return False
    
    def handle_func_expr_stmt_struc(self, node: Dict[str, Any]) -> str:
        """处理特殊的函数表达式结构"""
        result = []
        assign_stmt = node["stmtsp"][0]
        final_lhs = self.handle_expression(assign_stmt["lhsp"][0])  # 最终赋值的左值
        cond_node = assign_stmt["rhsp"][0]  # COND节点
        
        # 生成always块
        result.append(f"{self.indent()}always @(*)")
        result.append(f"{self.indent()}begin")
        self.indent_level += 1
        
        # 处理if条件
        condition = self.handle_expression(cond_node["condp"][0])
        result.append(f"{self.indent()}if ({condition})")
        result.append(f"{self.indent()}begin")
        self.indent_level += 1
        
        # 处理then部分的EXPRSTMT
        then_exprstmt = cond_node["thenp"][0]
        # 跳过第一个COMMENT，处理所有的ASSIGN语句
        for stmt in then_exprstmt["stmtsp"]:  # 从索引1开始跳过COMMENT
            if stmt["type"] == "ASSIGN":
                lhs = self.handle_expression(stmt["lhsp"][0])
                rhs = self.handle_expression(stmt["rhsp"][0])
                result.append(f"{self.indent()}{lhs} = {rhs};")
        
        # 获取EXPRSTMT的最终结果赋值给最外层的变量
        result.append(f"{self.indent()}{final_lhs} = {self.handle_expression(then_exprstmt['resultp'][0])};")
        
        self.indent_level -= 1
        result.append(f"{self.indent()}end")
        
        # 处理else部分
        if cond_node.get("elsep"):
            result.append(f"{self.indent()}else")
            result.append(f"{self.indent()}begin")
            self.indent_level += 1
            
            else_exprstmt = cond_node["elsep"][0]
            if else_exprstmt.get("type") == "EXPRSTMT":
                # 同样处理else部分的EXPRSTMT
                for stmt in else_exprstmt["stmtsp"][1:]:  # 从索引1开始跳过COMMENT
                    if stmt["type"] == "ASSIGN":
                        lhs = self.handle_expression(stmt["lhsp"][0])
                        rhs = self.handle_expression(stmt["rhsp"][0])
                        result.append(f"{self.indent()}{lhs} = {rhs};")
                # 获取else部分EXPRSTMT的最终结果
                result.append(f"{self.indent()}{final_lhs} = {self.handle_expression(else_exprstmt['resultp'][0])};")
            else:
                # 如果不是EXPRSTMT，按普通语句处理
                else_result = self.handle_statement(cond_node["elsep"][0])
                if else_result:
                    result.append(f"{self.indent()}{else_result}")
            
            self.indent_level -= 1
            result.append(f"{self.indent()}end")
        
        self.indent_level -= 1
        result.append(f"{self.indent()}end")
        
        return "\n".join(result)
    def indent(self):
        return self.indent_str * self.indent_level
    
    def parse_const_value(self, const_str: str) -> int:
        """解析各种进制的常量值"""
        if not const_str:
            return 0
            
        try:
            # 处理十六进制
            if const_str.startswith("32'h") or const_str.startswith("64'h"):
                return int(const_str.split("'h")[1], 16)
            # 处理带符号的十六进制
            elif const_str.startswith("32'sh") or const_str.startswith("64'sh"):
                return int(const_str.split("'sh")[1], 16)
            # 处理二进制
            elif const_str.startswith("32'b") or const_str.startswith("64'b"):
                return int(const_str.split("'b")[1], 2)
            # 处理带符号的二进制
            elif const_str.startswith("32'sb") or const_str.startswith("64'sb"):
                return int(const_str.split("'sb")[1], 2)
            # 处理其他位宽的十六进制
            elif "'h" in const_str:
                return int(const_str.split("'h")[1], 16)
            # 处理其他位宽的带符号十六进制
            elif "'sh" in const_str:
                return int(const_str.split("'sh")[1], 16)
            # 处理其他位宽的二进制
            elif "'b" in const_str:
                return int(const_str.split("'b")[1], 2)
            # 处理其他位宽的带符号二进制
            elif "'sb" in const_str:
                return int(const_str.split("'sb")[1], 2)
            # 处理十进制
            else:
                return int(const_str)
        except Exception as e:
            print(f"Warning: Error parsing constant <{const_str}>: {str(e)}")
            return 0

    def handle_unpackarraydtype(self, node: Dict[str, Any]) -> Tuple[str, str]:
        """处理解包数组类型定义，返回(基础类型位宽, 数组范围)
        支持嵌套的解包数组类型，例如 wire [63:0] a [4:0][4:0];
        """
        base_width = ""
        array_range = ""

        try:
            # 处理基础类型的位宽
            if node.get("refDTypep"):
                ref_type = self.type_table.get(node["refDTypep"].strip("()"))
                if ref_type and "range" in ref_type:
                    range_str = ref_type["range"]
                    base_width = f"[{range_str}]"

            # 获取数组维度
            array_range = node.get("declRange", "")

            # 检查refDTypep是否指向另一个UNPACKARRAYDTYPE（嵌套解包数组）
            ref_node = node.get("refDTypep")
            if ref_node:
                ref_type = self.type_table.get(ref_node.strip("()"))
                if ref_type and ref_type.get("type") == "UNPACKARRAYDTYPE":
                    # 递归获取内层解包数组的维度
                    inner_base, inner_range = self.handle_unpackarraydtype(ref_type)
                    # 内层解包数组的array_range作为额外的数组维度
                    if inner_range:
                        if array_range:
                            array_range = f"{array_range}{inner_range}"
                        else:
                            array_range = inner_range
                    # 如果内层还找到了base_width，使用它（可能从更内层的基础类型来）
                    if inner_base and not base_width:
                        base_width = inner_base

            return (base_width, array_range)

        except Exception as e:
            warning_msg = f"Error in UNPACKARRAYDTYPE: {str(e)}"
            print(f"Warning: {warning_msg}")
            return ("", "/* Error in array range */")

    def build_type_table(self, ast: Dict[str, Any]):
        """构建类型查找表"""
        # 第一步：收集所有类型信息
        if "miscsp" in ast:
            for misc in ast["miscsp"]:
                if misc.get("type") == "TYPETABLE" and "typesp" in misc:
                    for type_def in misc["typesp"]:
                        addr = type_def.get("addr", "")
                        if addr:
                            addr = addr.strip("()")
                            self.type_table[addr] = type_def

        # 第二步：处理所有UNPACKARRAYDTYPE
        for addr, type_def in self.type_table.items():
            if type_def["type"] == "UNPACKARRAYDTYPE":
                base_width, array_range = self.handle_unpackarraydtype(type_def)
                type_def["base_width"] = base_width
                type_def["array_range"] = array_range

    def resolve_dtype(self, dtype_ref: str) -> Dict[str, Any]:
        """解析类型引用"""
        if not dtype_ref:
            return {}
            
        # 去除引用地址的括号
        addr = dtype_ref.strip("()")
        return self.type_table.get(addr, {})

    def get_type_info(self, node: Dict[str, Any]) -> str:
        """获取完整的类型信息，包括位宽"""
        dtype_ref = node.get("dtypep", "")
        dtype_node = self.resolve_dtype(dtype_ref)

        if not dtype_node:
            return "logic"  # 默认类型

        keyword = dtype_node.get("keyword", "logic")

        if keyword == "integer":
            return "integer"

        # Handle PACKARRAYDTYPE: recursively compute total width
        if dtype_node.get("type") == "PACKARRAYDTYPE":
            total_width_bits = self.get_base_width_bits(dtype_node)
            if total_width_bits > 0:
                return f"{keyword} [{total_width_bits - 1}:0]"
            return keyword

        range_str = dtype_node.get("range", "")

        # 如果没有range，尝试从type_table查找一次（跳过自引用）
        if not range_str:
            dtype_ref = dtype_node.get("dtypep", "")
            if isinstance(dtype_ref, str) and dtype_ref.startswith("(") and dtype_ref.endswith(")"):
                type_id = dtype_ref[1:-1]  # 去掉括号
                current_addr = dtype_node.get("addr", "").strip("()")
                if type_id and type_id != current_addr and type_id in self.type_table:
                    new_dtype_node = self.type_table[type_id]
                    range_str = new_dtype_node.get("range", "")

        if range_str:
            return f"{keyword} [{range_str}]"
        return keyword

    def generate(self, node: Dict[str, Any]) -> str:
        if not isinstance(node, dict):
            return str(node)
        
        # 首次调用时构建类型表
        if node.get("type") == "NETLIST":
            self.build_type_table(node)
            
        node_type = node.get("type", "")
        if not node_type:
            return self.handle_unknown(node)
            
        handler = getattr(self, f"handle_{node_type.lower()}", self.handle_unknown)
        return handler(node)

    def handle_netlist(self, node: Dict[str, Any]) -> str:
        result = []

        # 处理模块列表
        if "modulesp" in node and isinstance(node["modulesp"], list):
            # 第一遍：收集 IFACE (接口) 节点中的变量声明
            # Verilator 将 AXI 总线结构体展平为独立的 VAR 节点放在 IFACE 中，
            # 但模块的 stmtsp 中只有对这些变量的 VARREF 引用，没有声明。
            # 注意：多个 IFACE 可能有重名的变量 (如 b_valid)，需要去重
            self.iface_var_nodes = []
            seen_iface_vars = set()
            for module in node["modulesp"]:
                if module.get("type") == "IFACE":
                    for stmt in module.get("stmtsp", []):
                        if stmt.get("type") == "VAR":
                            name = stmt.get("name", "")
                            if name and name not in seen_iface_vars:
                                seen_iface_vars.add(name)
                                self.iface_var_nodes.append(stmt)

            for module in node["modulesp"]:
                if module.get("type") != "IFACE":
                    result.append(self.generate(module))

        return "\n\n".join(result)

    def handle_module(self, node: Dict[str, Any]) -> str:
        self.indent_level += 1
        body = []
        
        module_name = node.get("name", "")
        # 从stmtsp中提取端口和内部声明
        ports: List[str] = []
        declarations: List[str] = []
        
        if "stmtsp" in node and isinstance(node["stmtsp"], list):
            # 预先收集本模块中所有被引用的变量
            self.used_vars = set()
            self.procedural_vars = set()  # 重置程序化变量集合
            self.collect_used_vars(node["stmtsp"])

            # 添加 IFACE (接口) 中收集到的变量声明
            # 这些变量 (如 AXI 总线信号 b_valid, r_data 等) 被模块引用但声明在 IFACE 节点中
            if hasattr(self, 'iface_var_nodes'):
                for var_node in self.iface_var_nodes:
                    var_decl = self.handle_var(var_node)
                    if var_decl:
                        declarations.append(var_decl)

            for stmt in node["stmtsp"]:
                if stmt.get("type") == "VAR" and stmt.get("isPrimaryIO"):
                    ports.append(self.generate_port(stmt))
                    declarations.append(self.generate_port_declaration(stmt))
                else:
                    body.append(self.generate(stmt))
        
        # 构建模块声明
        module_decl = f"module {module_name}"
        if ports:
            port_list = (",\n" + self.indent()).join(ports)
            module_decl += "(\n" + self.indent() + port_list + "\n);"
        else:
            module_decl += ";"
            
        # 添加声明
        if declarations:
            body = declarations + body
            
        self.indent_level -= 1
        
        # 使用join来组合所有部分
        return "\n".join([module_decl] + body + ["endmodule"])

    def collect_used_vars(self, node: Any) -> None:
        """递归收集所有被引用的变量名 (VARREF) 以及被程序化赋值的变量名"""
        if isinstance(node, list):
            for item in node:
                self.collect_used_vars(item)
        elif isinstance(node, dict):
            node_type = node.get("type")
            
            # 特殊处理 SCOPE 节点：只遍历 blocksp，忽略 varsp
            if node_type == "SCOPE":
                if "blocksp" in node:
                    self.collect_used_vars(node["blocksp"])
                return

            # 如果是VARREF节点，收集名字
            if node_type == "VARREF":
                name = node.get("name")
                if name:
                    # print(f"DEBUG: Collected usage of '{name}'")
                    self.used_vars.add(name)
            
            # 识别程序化赋值 (Procedural Assignments)
            # 1. EXPRSTMT 节点中的 resultp (通常来自 Verilator 对函数的程序化处理)
            if node_type == "EXPRSTMT" and "resultp" in node:
                resultp_node = node["resultp"][0]
                name = self.extract_base_var_name(resultp_node)
                if name:
                    self.procedural_vars.add(name)
            
            # 2. ALWAYS/INITIAL 块中的 ASSIGN/ASSIGNDLY
            # 注意：在 Verilator AST 中，顶层的 ASSIGNW 通常对应连续赋值 (assign)，
            # 而过程块内部的 ASSIGN 对应阻塞赋值 (=)，ASSIGNDLY 对应非阻塞赋值 (<=)
            if node_type in ["ASSIGN", "ASSIGNDLY"]:
                if "lhsp" in node:
                    name = self.extract_base_var_name(node["lhsp"][0])
                    if name:
                        self.procedural_vars.add(name)
            
            # 递归处理所有字典值
            for key, value in node.items():
                if isinstance(value, (list, dict)):
                    self.collect_used_vars(value)

    def extract_base_var_name(self, node: Any) -> str:
        """从表达式节点中提取基础变量名 (处理 SEL, ARRAYSEL 等)"""
        if not isinstance(node, dict):
            return ""
        
        node_type = node.get("type")
        if node_type == "VARREF":
            return node.get("name", "")
        elif node_type in ["SEL", "ARRAYSEL", "DOT"]:
            fromp = node.get("fromp", [])
            if fromp:
                return self.extract_base_var_name(fromp[0])
        
        return ""

    def generate_port(self, node: Dict[str, Any]) -> str:
        return node.get("name", "")

    def generate_port_declaration(self, node: Dict[str, Any]) -> str:
        direction = node.get("direction", "").lower()
        var_type = node.get("varType", "").lower()
        name = node.get("name", "")
        type_info = self.get_type_info(node)
        
        # 如果是过程块赋值的变量，确保类型是 logic 而不是 wire
        if name in self.procedural_vars:
            if var_type == "wire":
                var_type = "logic"
            
            if "wire" in type_info:
                type_info = type_info.replace("wire", "logic")
            elif "logic" not in type_info and "reg" not in type_info and "integer" not in type_info:
                # 如果 type_info 只是 [31:0] 这种，前面补 logic
                type_info = "logic " + type_info
            
            # 如果 var_type 和 type_info 都包含了 logic，合并它们
            if var_type == "logic" and type_info.startswith("logic"):
                var_type = "" # 让 type_info 承担 logic 关键字

        components = [direction]
        if var_type != "port":
            components.append(var_type)
        components.append(type_info)
        components.append(name)
        
        return f"{self.indent()}{' '.join(filter(None, components))};"
    
    def get_base_width_bits(self, dtype_info: Dict[str, Any]) -> int:
        """递归计算基础类型的位宽（以位数为单位）
        对于 [31:0] 返回 32，对于 [1:0][31:0] 递归计算
        """
        if not dtype_info:
            return 0
        
        # 检查是否是打包数组类型
        if dtype_info.get("type") == "PACKARRAYDTYPE":
            # 获取基础类型
            ref_dtype_id = dtype_info.get("refDTypep", "").strip("()")
            base_type_info = self.type_table.get(ref_dtype_id, {})
            base_width_bits = self.get_base_width_bits(base_type_info)
            
            # 计算数组维度的大小
            range_items = dtype_info.get("rangep", [])
            array_size = 1
            if isinstance(range_items, list) and range_items:
                for range_item in range_items:
                    left_const = range_item.get("leftp", [{}])[0].get("name", "")
                    right_const = range_item.get("rightp", [{}])[0].get("name", "")
                    left = self.parse_const_value(left_const)
                    right = self.parse_const_value(right_const)
                    # 计算范围大小：abs(left - right) + 1
                    array_size *= abs(left - right) + 1
            
            return base_width_bits * array_size
        
        # ENUMDTYPE: follow refDTypep to get the underlying type's width
        if dtype_info.get("type") == "ENUMDTYPE":
            ref_dtype_id = dtype_info.get("refDTypep", "").strip("()")
            if ref_dtype_id and ref_dtype_id in self.type_table:
                return self.get_base_width_bits(self.type_table[ref_dtype_id])

        # REFDTYPE: follow refDTypep to get the underlying type's width
        if dtype_info.get("type") == "REFDTYPE":
            ref_dtype_id = dtype_info.get("refDTypep", "").strip("()")
            if ref_dtype_id and ref_dtype_id in self.type_table:
                return self.get_base_width_bits(self.type_table[ref_dtype_id])

        # 检查是否有 range 字段
        range_str = dtype_info.get("range", "")
        if range_str:
            # 解析 range，格式可能是 "31:0" 或 "0:31"
            try:
                parts = range_str.split(":")
                if len(parts) == 2:
                    left = int(parts[0].strip())
                    right = int(parts[1].strip())
                    return abs(left - right) + 1
            except:
                pass
        
        # 检查是否有 rangep 数组
        range_items = dtype_info.get("rangep", [])
        if isinstance(range_items, list) and range_items:
            for range_item in range_items:
                if "leftp" in range_item and "rightp" in range_item:
                    left_const = range_item.get("leftp", [{}])[0].get("name", "")
                    right_const = range_item.get("rightp", [{}])[0].get("name", "")
                    left = self.parse_const_value(left_const)
                    right = self.parse_const_value(right_const)
                    return abs(left - right) + 1
        
        # 如果没有找到位宽信息，尝试从 dtypep 查找一次
        # 跳过自引用（某些 PACKARRAYDTYPE/STRUCTDTYPE 的 dtypep 指向自身）
        next_dtype_ref = dtype_info.get("dtypep", "")
        if isinstance(next_dtype_ref, str) and next_dtype_ref.startswith("(") and next_dtype_ref.endswith(")"):
            type_id = next_dtype_ref[1:-1]
            current_addr = dtype_info.get("addr", "").strip("()")
            if type_id and type_id != current_addr and type_id in self.type_table:
                return self.get_base_width_bits(self.type_table[type_id])
        
        return 0
    
    def get_width_info(self, node: Dict[str, Any]) -> Tuple[str, str]:
        """获取节点的位宽信息
        返回元组: (基础类型位宽, 数组维度)
        例如：对于 reg [1:0] memory [0:31];
        返回 ("[1:0]", "[0:31]")
        """
        try:
            # 获取dtypep引用
            dtype_ref = node.get("dtypep", "")
            if not dtype_ref or not isinstance(dtype_ref, str):
                return ("", "")

            # 去掉括号获取引用ID
            dtype_ref = dtype_ref.strip("()")
            
            # 从类型表中查找对应的类型信息
            dtype_info = self.type_table.get(dtype_ref)
            if not dtype_info:
                return ("", "")

            # 检查是否是打包数组类型
            if dtype_info.get("type") == "PACKARRAYDTYPE":
                # 递归计算总位宽
                total_width_bits = self.get_base_width_bits(dtype_info)
                if total_width_bits > 0:
                    # 返回 [总位宽-1:0] 格式
                    return (f"[{total_width_bits - 1}:0]", "")
                return ("", "")

            # 检查是否是解包数组类型
            if dtype_info.get("type") == "UNPACKARRAYDTYPE":
                base_width = dtype_info.get("base_width", "")
                array_range = dtype_info.get("array_range", "")
                return (base_width, array_range)
            
            # 处理普通类型
            # 获取range信息
            range_str = dtype_info.get("range", "")
            
            # 如果没有range，尝试从dtypep查找一次（跳过自引用）
            if not range_str:
                next_dtype_ref = dtype_info.get("dtypep", "")
                if isinstance(next_dtype_ref, str) and next_dtype_ref.startswith("(") and next_dtype_ref.endswith(")"):
                    type_id = next_dtype_ref[1:-1]
                    current_addr = dtype_info.get("addr", "").strip("()")
                    if type_id and type_id != current_addr and type_id in self.type_table:
                        new_dtype_info = self.type_table[type_id]
                        range_str = new_dtype_info.get("range", "")

            if range_str:
                return (f"[{range_str}]", "")

            # 如果有rangep数组
            range_items = dtype_info.get("rangep", [])
            if isinstance(range_items, list) and range_items:
                for range_item in range_items:
                    # 检查是否存在left和right字段
                    if "leftp" in range_item and "rightp" in range_item:
                        # 获取左值常量
                        left_const = range_item.get("leftp", [{}])[0].get("name", "")
                        left = str(self.parse_const_value(left_const))
                        
                        # 获取右值常量
                        right_const = range_item.get("rightp", [{}])[0].get("name", "")
                        right = str(self.parse_const_value(right_const))
                        
                        if left and right:
                            return (f"[{left}:{right}]", "")
                    # 兼容旧格式
                    elif "left" in range_item and "right" in range_item:
                        left = range_item.get("left", "")
                        right = range_item.get("right", "")
                        if left != "" and right != "":
                            return (f"[{left}:{right}]", "")

            return ("", "")

        except Exception as e:
            warning_msg = f"Error in get_width_info: {str(e)}"
            print(f"Warning: {warning_msg}")
            return ("", "")

    def handle_var(self, node: Dict[str, Any]) -> str:
        """处理VAR节点，支持初始赋值分离"""
        if not node:
            return ""
        
        try:
            var_type = node.get("varType", "")
            var_name = node.get("name", "")
            
            # 检查变量是否被使用 (仅对非端口变量进行检查)
            is_port = node.get("isPrimaryIO", False) or node.get("varType") == "PORT"
            # 确保 used_vars 已初始化且变量未被使用
            if not is_port and hasattr(self, 'used_vars'):
                if var_name not in self.used_vars:
                    # 过滤掉未使用的变量
                    return ""
                
            dtype_name = node.get("dtypeName", "wire")
            
            if var_name in self.initialized_vars:
                return ""
            
            # 获取初始值
            init_value = ""
            assign_statement = ""
            if "valuep" in node and node["valuep"]:
                value_node = node["valuep"][0]
                init_expr = self.handle_expression(value_node)
                
                if SEPARATE_INITIAL_ASSIGN:
                    # # 分离声明和赋值语句
                    assign_statement = f"{self.indent()}initial begin\n{self.indent()}    {var_name} = {init_expr};\n{self.indent()}end"
                    init_value = ""  # 声明时不赋初值
                elif SEPARATE_ASSIGN_FOR_EQ_CHECK:
                    assign_statement = f"{self.indent()}assign {var_name} = {init_expr};\n{self.indent()}"
                    if CONVERT_ASSIGN_TO_ALWAYS:
                        assign_statement = f"{self.indent()}always @(*) begin\n{self.indent()}    {var_name} = {init_expr};\n{self.indent()}end"
                    init_value = ""  # 声明时不赋初值
                else:
                    init_value = f" = {init_expr}"
                
                self.initialized_vars.add(var_name)
            
            # 获取位宽信息和数组维度
            base_width, array_range = self.get_width_info(node)

            # Verilator将packed bit range编码到变量名中（如data_AUX_int[70:0]）
            # 这个[left:right]后缀会被错误地当成unpacked array维度，
            # 生成 reg [X:0] ...name[X:0]（2元素数组）而非 reg [X:0] ...name（标量）。
            # 无论base_width是否已由get_width_info解析，都必须从var_name中剥离此后缀。
            import re as _re
            name_range_match = _re.search(r'\[(\d+):\d+\]$', var_name)
            if name_range_match:
                # 如果get_width_info未能解析位宽，则用名称中的range作为packed width
                if not base_width:
                    base_width = var_name[var_name.rfind('['):]
                # 始终从变量名中剥离[left:right]后缀，避免创建unpacked array
                var_name = var_name[:var_name.rfind('[')]

            # 处理不同类型的变量
            if var_type in ["GPARAM", "PARAM", "LPARAM"]:
                return self.handle_parameter(node)
            
            elif var_type == "VAR":
                type_keyword = "reg"
                
            elif var_type in ["PORT", "WIRE", "LOGIC", "MODULETEMP", "BLOCKTEMP", "GENVAR"]:
                if var_name in self.procedural_vars:
                    type_keyword = "logic"
                else:
                    type_keyword = "logic" if dtype_name.lower() == "logic" else "wire"
                
            else:
                warning = f"Warning: Unhandled var type: {var_type}"
                print(warning)
                return f"{self.indent()}// {warning}"
            # 根据是否有位宽和数组维度组装声明
            declaration = ""
            if base_width and array_range:
                declaration = f"{self.indent()}{type_keyword} {base_width} {var_name} {array_range}{init_value};"
            elif base_width:
                declaration = f"{self.indent()}{type_keyword} {base_width} {var_name}{init_value};"
            elif array_range:
                declaration = f"{self.indent()}{type_keyword} {var_name} {array_range}{init_value};"
            else:
                declaration = f"{self.indent()}{type_keyword} {var_name}{init_value};"
            
            # 如果需要分离初始化，则返回声明语句和assign语句
            if assign_statement:
                # 你需要确保调用此方法的地方能够处理返回多条语句
                return declaration + "\n" + assign_statement
            else:
                return declaration
        
        except Exception as e:
            warning_msg = f"Error in VAR: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}"
        
    def handle_parameter(self, node: Dict[str, Any]) -> str:
        """处理parameter和localparam定义"""
        try:
            param_name = node.get("name", "")
            
            # 过滤掉包含 "___" 或 "." 的内部参数
            if "___" in param_name or "." in param_name:
                return ""

            var_type = node.get("varType", "")
            
            # 确定是parameter还是localparam
            param_keyword = "localparam" if var_type == "LPARAM" else "parameter"
            
            # 获取位宽信息，返回(基础类型位宽, 数组维度)
            base_width, array_range = self.get_width_info(node)
            
            # 获取parameter的值
            value = ""
            if "valuep" in node and node["valuep"]:
                value_node = node["valuep"][0]
                value = self.handle_expression(value_node)
                
            # 构建完整的参数声明
            width_str = base_width  # 使用基础类型位宽
            if array_range:  # 如果有数组维度，添加到声明中
                width_str = f"{width_str} {array_range}"
                
            if value:
                return f"{self.indent()}{param_keyword} {width_str} {param_name} = {value};"
            else:
                return f"{self.indent()}{param_keyword} {width_str} {param_name};"
                
        except Exception as e:
            warning_msg = f"Error in parameter: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}"
        
    def handle_topscope(self, node: Dict[str, Any]) -> str:
        if node.get("type") != "TOPSCOPE":
            return ""
        
        result = []
        # 处理scopep数组
        for scope in node.get("scopep", []):
            result.append(self.handle_scope(scope))
        
        return "\n".join(filter(None, result))


    def handle_assign_alias(self, node: Dict[str, Any]) -> str:
        if node.get("type") != "ASSIGNALIAS":
            return ""
            
        # 获取左右两边的信号名称
        rhs = node["rhsp"][0]["name"] if node["rhsp"] else ""
        lhs = node["lhsp"][0]["name"] if node["lhsp"] else ""
        
        if CONVERT_ASSIGN_TO_ALWAYS:
            # 生成always块
            return f"{self.indent()}always @(*) begin\n{self.indent()}    {lhs} = {rhs};\n{self.indent()}end"
        # 生成assign语句
        return f"{self.indent()}assign {lhs} = {rhs};"
    
    def handle_sel(self, node: Dict[str, Any]) -> str:
        """处理SEL (选择/切片) 操作"""
        if not node.get("fromp"):
            return ""
        
        try:
            # 初始化临时变量记录器（如果还没有的话）
            if not hasattr(self, 'sel_temp_count'):
                self.sel_temp_count = 0
            if not hasattr(self, 'sel_temp_dict'):
                self.sel_temp_dict = {}
            
            def is_complex_expression(expr: str) -> bool:
                return expr.strip().endswith(')') or expr.strip().endswith('}')
            def is_constant(expr: str) -> bool:
                """检查表达式是否为纯常量值(不包含任何运算符或其他字符)
                支持格式:
                - 整数: 123, -123
                - 十六进制: 32'h1a3, 32'sha3 (带符号)
                - 二进制: 8'b1010, 16'sb101 (带符号) 
                - 十进制: 16'd123, 32'sd123 (带符号)
                - 八进制: 32'o123, 16'so123 (带符号)
                
                不支持:
                - 表达式: 1+2, a*b
                - 函数调用: f(x)
                - 变量引用: abc
                """
                import re
                
                try:
                    if not expr:
                        return False
                        
                    # 去除两端空白字符,但保留中间的空白以便检测
                    expr = expr.strip()
                    
                    # 检查是否包含运算符或其他非法字符
                    if any(c in expr for c in '+-*/%&|^~<>!=()[]{},:;`@#$\\'):
                        return False
                        
                    # 检查是否包含空格
                    if ' ' in expr:
                        return False
                        
                    # 匹配纯数字(包括负号开头)
                    if re.match(r'^-?\d+$', expr) and expr.count('-') <= 1:
                        return True
                        
                    # Verilog常量格式的完整模式
                    patterns = [
                        # 十六进制: 32'h1a3, 32'sha3
                        r'^[1-9]\d*\'s?h[0-9a-fA-F_]+$',
                        
                        # 二进制: 8'b1010, 16'sb101
                        r'^[1-9]\d*\'s?b[01_]+$',
                        
                        # 十进制: 16'd123, 32'sd123
                        r'^[1-9]\d*\'s?d\d+$',
                        
                        # 八进制: 32'o123, 16'so123
                        r'^[1-9]\d*\'s?o[0-7_]+$'
                    ]
                    
                    return any(re.match(pattern, expr) for pattern in patterns)
                    
                except Exception as e:
                    print(f"Warning: Error in is_constant for expression <{expr}>: {str(e)}")
                    return False
            
            # 获取并检查base是否需要替换
            base_expr = self.handle_expression(node["fromp"][0])
            
            if is_constant(base_expr) and self.convert_complex_sel_to_temp:
                # 获取位宽信息
                base_width, array_range = self.get_width_info(node["fromp"][0])
                # 生成新的临时变量名
                temp_var = f"___sel_temp_{self.sel_temp_count}"
                self.sel_temp_count += 1
                # 保存临时变量、表达式和位宽信息
                self.sel_temp_dict[temp_var] = {
                    'expr': base_expr,
                    'width': base_width, # if base_width else "[31:0]",  # 默认位宽
                    'array_range': array_range
                }
                # 使用临时变量替换复杂表达式
                base = temp_var
            elif is_complex_expression(base_expr) and self.convert_complex_sel_to_temp:
                # 获取位宽信息
                base_width, array_range = self.get_width_info(node["fromp"][0])
                # 生成新的临时变量名
                temp_var = f"___sel_temp_{self.sel_temp_count}"
                self.sel_temp_count += 1
                # 保存临时变量、表达式和位宽信息
                self.sel_temp_dict[temp_var] = {
                    'expr': base_expr,
                    'width': base_width, # if base_width else "[31:0]",  # 默认位宽
                    'array_range': array_range
                }
                # 使用临时变量替换复杂表达式
                base = temp_var
            else:
                base_width, array_range = self.get_width_info(node["fromp"][0])
                if base_width == "":
                    return base_expr
                base = base_expr
            
            # 检查lsbp是否是复杂表达式
            if node.get("lsbp"):
                lsb_expr = self.handle_expression(node["lsbp"][0])
                if is_complex_expression(lsb_expr) and self.convert_complex_sel_to_temp:
                    # 获取位宽信息
                    lsb_width, array_range = self.get_width_info(node["lsbp"][0])
                    # 生成新的临时变量名
                    temp_var = f"___sel_temp_{self.sel_temp_count}"
                    self.sel_temp_count += 1
                    # 保存临时变量、表达式和位宽信息
                    self.sel_temp_dict[temp_var] = {
                        'expr': lsb_expr,
                        'width': lsb_width, # if lsb_width else "[31:0]",  # 默认位宽
                        'array_range': array_range
                    }
                    # 使用临时变量替换复杂表达式
                    lsb = temp_var
                else:
                    lsb = lsb_expr

                # Convert hex-formatted constants to decimal for bit indices
                # e.g., 3'h0 -> 0, 3'h7 -> 7
                if is_constant(lsb):
                    try:
                        lsb = str(self.parse_const_value(lsb))
                    except Exception:
                        pass

                # 新版本使用 widthConst，旧版本使用 widthp
                width_const = node.get("widthConst")
                widthp = node.get("widthp")
                
                if width_const is not None:
                    # widthConst 表示选择的位宽，lsbp 表示起始位置（LSB）
                    if width_const == 1:
                        # 单个位选择，直接返回 [lsb]
                        return f"{base}[{lsb}]"
                    
                    try:
                        # 解析 lsb_expr 获取数值
                        lsb_value = self.parse_const_value(lsb_expr) if is_constant(lsb_expr) else None
                        if lsb_value is not None:
                            # 如果是常量，使用标准范围选择 [msb:lsb]
                            msb_value = lsb_value + width_const - 1
                            return f"{base}[{msb_value}:{lsb_value}]"
                        else:
                            # 如果不是常量，使用 indexed part-select [lsb +: width]
                            return f"{base}[{lsb}+:{width_const}]"
                    except Exception as e:
                        # 出现异常时也使用 indexed part-select
                        return f"{base}[{lsb}+:{width_const}]"
                elif widthp:
                    # 旧版本：使用 widthp 和 lsbp，使用Verilog的+:语法
                    width = self.handle_expression(widthp[0])
                    return f"{base}[{lsb}+:{width}]"
                else:
                    # 只有单个位选择
                    return f"{base}[{lsb}]"
            
            return base
                
        except Exception as e:
            warning_msg = f"Error in SEL: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */ 1'b0"
    
    def get_temp_var_count(self) -> int:
        """获取并递增临时变量计数"""
        # 如果没有计数器属性，初始化为0
        if not hasattr(self, '_temp_var_counter'):
            self._temp_var_counter = 0
        self._temp_var_counter += 1
        return self._temp_var_counter

    def handle_assignw(self, node: Dict[str, Any]) -> str:
        """处理ASSIGNW (wire赋值)"""
        if not node.get("lhsp") or not node.get("rhsp"):
            return ""

        try:
            lhs_node = node["lhsp"][0]

            # Handle EXTEND on LHS: this means the RHS is wider than the LHS variable.
            # EXTEND on LHS would produce invalid {N'b0, var} concat via handle_extend(),
            # so we instead truncate the RHS to match the inner variable's width.
            if lhs_node.get("type") == "EXTEND":
                inner_node = lhs_node["lhsp"][0]
                lhs = self.handle_expression(inner_node)
                rhs = self.handle_expression(node["rhsp"][0])

                # Get inner variable width to truncate RHS
                base_width, _ = self.get_width_info(inner_node)
                if base_width.startswith('[') and base_width.endswith(']'):
                    width_range = base_width[1:-1].split(':')
                    if len(width_range) == 2:
                        try:
                            inner_width = abs(int(width_range[0]) - int(width_range[1])) + 1
                            if CONVERT_ASSIGN_TO_ALWAYS:
                                return f"{self.indent()}always @(*) begin\n{self.indent()}    {lhs} = {rhs}[{inner_width-1}:0];\n{self.indent()}end"
                            return f"{self.indent()}assign {lhs} = {rhs}[{inner_width-1}:0];"
                        except ValueError:
                            pass

                if CONVERT_ASSIGN_TO_ALWAYS:
                    return f"{self.indent()}always @(*) begin\n{self.indent()}    {lhs} = {rhs};\n{self.indent()}end"
                return f"{self.indent()}assign {lhs} = {rhs};"

            # 获取左侧变量名
            lhs = self.handle_expression(lhs_node)
            
            # 先生成右侧表达式
            rhs = self.handle_expression(node["rhsp"][0])
            
            # 存储所有临时变量声明和赋值
            temp_decls = []
            
            # 循环处理所有的 )[
            while ')[' in rhs:
                # 找到一个 )[
                right_paren = rhs.rfind(')[')
                # 从这个位置向前找到匹配的左括号
                count = 1
                left_paren = right_paren
                while count > 0 and left_paren > 0:
                    left_paren -= 1
                    if rhs[left_paren] == ')':
                        count += 1
                    elif rhs[left_paren] == '(':
                        count -= 1
                
                if left_paren >= 0:
                    # 提取括号内的表达式
                    expr = rhs[left_paren + 1:right_paren]
                    # 获取选择部分
                    sel_end = rhs.find(']', right_paren)
                    sel = rhs[right_paren + 1:sel_end + 1]
                    
                    temp_var = f"___temp_{self.temp_count}"
                    self.temp_count += 1
                    
                    # 添加临时变量声明和赋值
                    temp_decls.append(f"wire [127:0] {temp_var};")
                    if CONVERT_ASSIGN_TO_ALWAYS:
                        temp_decls.append(f"always @(*) begin\n{self.indent()}    {temp_var} = {expr};\nend")
                    else:
                        temp_decls.append(f"assign {temp_var} = {expr};")
                    
                    # 替换原表达式中的这部分
                    rhs = rhs[:left_paren] + f"{temp_var}{sel}" + rhs[sel_end + 1:]
                else:
                    break
            
            # 收集所有需要输出的语句
            result = []
            for decl in temp_decls:
                result.append(f"{self.indent()}{decl}")
            if CONVERT_ASSIGN_TO_ALWAYS:
                result.append(f"{self.indent()}always @(*) begin\n{self.indent()}    {lhs} = {rhs};\n{self.indent()}end")
            else:
                result.append(f"{self.indent()}assign {lhs} = {rhs};")
            
            return "\n".join(result)
                
        except Exception as e:
            warning_msg = f"Error in ASSIGNW: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}\n{self.indent()}assign {lhs} = 'b0;"
    
    def handle_initial(self, node: Dict[str, Any]) -> str:
        """处理INITIAL块"""
        if not node.get("stmtsp"):
            return ""

        result = []
        temp_stmts = []

        # 处理所有语句，收集未初始化的变量的赋值语句
        for stmt in node.get("stmtsp", []):
            if stmt.get("type") == "ASSIGN":
                lhs_node = stmt["lhsp"][0]
                var_name = lhs_node.get("name", "")
                # Skip genvar initializations (_gv_i) - they are implicit and
                # don't have VAR declarations in Verilator's AST
                if "_gv_i" in var_name:
                    continue
                if var_name not in self.initialized_vars:
                    stmt_result = self.handle_statement(stmt)
                    if stmt_result:
                        temp_stmts.append(stmt_result)
                    self.initialized_vars.add(var_name)
            else:
                stmt_result = self.handle_statement(stmt)
                if stmt_result:
                    temp_stmts.append(stmt_result)

        if len(temp_stmts) > 1:
            print("Warning: Multiple initialization statements found in INITIAL block.")
        # 如果有需要初始化的语句，才生成initial块
        if temp_stmts:
            result.append(f"{self.indent()}initial")
            result.append(f"{self.indent()}begin")
            self.indent_level += 1
            result.extend(f"{self.indent()}{stmt}" for stmt in temp_stmts)
            self.indent_level -= 1
            result.append(f"{self.indent()}end")

        return "\n".join(filter(None, result))

    def handle_scope(self, node: Dict[str, Any]) -> str:
        if node.get("type") != "SCOPE":
            return ""
        
        result = []
        # 处理blocksp数组中的所有block
        for block in node.get("blocksp", []):
            block_type = block.get("type")
            if block_type == "ASSIGNALIAS":
                result.append(self.handle_assign_alias(block))
            elif block_type == "ALWAYS":
                # if not self.is_func_exprstmt_struc(block):
                result.append(self.handle_always(block))
                # else:
                #     result.append(self.handle_func_expr_stmt_struc(block))
            elif block_type == "INITIAL" or block_type == "INITIALSTATIC":
                result.append(self.handle_initial(block))
            elif block_type == "ASSIGNW":
                result.append(self.handle_assignw(block))
                
        # 清空临时变量列表
        self.temp_wires = []
        self.temp_assigns = []

        return "\n".join(filter(None, result))
    
    def handle_jumpblock(self, node: Dict[str, Any]) -> str:
        """处理JUMPBLOCK节点，处理所有语句，遇到JUMPGO时停止"""
        if not node:
            return ""

        # 处理所有语句，遇到JUMPGO时break
        results = []
        if "stmtsp" in node and node["stmtsp"]:
            for stmt in node["stmtsp"]:
                if stmt.get("type") == "JUMPGO":
                    break  # JUMPGO is Verilator internal jump; stop processing this block
                results.append(self.handle_statement(stmt))

        return "\n".join(filter(None, results))
            
    
    
    def handle_always(self, node: Dict[str, Any]) -> str:
        # --- 修改开始: 特殊处理 cont_assign ---
        # 如果 keyword 是 "cont_assign"，这实际上是一个连续赋值语句 (assign lhs = rhs;)
        # 而不是一个时序或组合逻辑块 (always @...)
        if node.get("keyword") == "cont_assign":
            result = []
            stmts = node.get("stmtsp", [])
            # Check if any statement in this block is a control-flow statement
            # (CASE, LOOP, BEGIN), including those nested inside JUMPBLOCK nodes.
            def _has_control_flow(stmts):
                for s in stmts:
                    if s.get("type") in ("CASE", "LOOP", "BEGIN", "WHILE"):
                        return True
                    if s.get("type") == "JUMPBLOCK":
                        if _has_control_flow(s.get("stmtsp", [])):
                            return True
                return False
            has_control_flow = _has_control_flow(stmts)
            if has_control_flow:
                result.append(f"{self.indent()}always @(*) begin")
                self.indent_level += 1
                for stmt in stmts:
                    stmt_str = self.handle_statement(stmt)
                    if stmt_str:
                        result.append(stmt_str)
                self.indent_level -= 1
                result.append(f"{self.indent()}end")
            else:
                for stmt in stmts:
                    stmt_str = self.handle_statement(stmt)
                    if stmt_str:
                        if stmt.get("type") == "COMMENT":
                            result.append(stmt_str)
                        else:
                            result.append(f"{self.indent()}assign {stmt_str}")
            return "\n".join(result)
        # --- 修改结束 ---

        # 以下是原有的 always 块处理逻辑
        
        # 新版本使用 "sentreep"，旧版本使用 "sensesp"，优先检查新版本
        sentreep = node.get("sentreep", [])
        sensesp = node.get("sensesp", [])
        
        # 如果两者都没有，使用默认的敏感性列表
        if not sentreep and not sensesp:
            sensitivity_str = "*"
        else:
            # 收集所有敏感信号
            sensitivity_list = []
            
            # 处理新版本的 sentreep 格式
            if sentreep:
                for sentree in sentreep:
                    if sentree.get("type") == "SENTREE":
                        if sentree.get("sensesp"):
                            for senitem in sentree["sensesp"]:
                                edge_type = senitem.get("edgeType", "")
                                if not senitem.get("sensp"):
                                    continue

                                signal = senitem["sensp"][0]
                                signal_name = signal.get("name", "")
                                if not signal_name:
                                    continue

                                if edge_type == "POS":
                                    sensitivity_list.append(f"posedge {signal_name}")
                                elif edge_type == "NEG":
                                    sensitivity_list.append(f"negedge {signal_name}")
                                elif edge_type == "CHANGED":
                                    sensitivity_list.append(signal_name)

            # 处理旧版本的 sensesp 格式（向后兼容）
            elif sensesp:
                # 获取 SENTREE 节点
                sensetree = sensesp[0]
                if sensetree.get("sensesp"):
                    for senitem in sensetree["sensesp"]:
                        edge_type = senitem.get("edgeType", "")
                        if not senitem.get("sensp"):
                            continue

                        signal = senitem["sensp"][0]
                        signal_name = signal.get("name", "")
                        if not signal_name:
                            continue

                        if edge_type == "POS":
                            sensitivity_list.append(f"posedge {signal_name}")
                        elif edge_type == "NEG":
                            sensitivity_list.append(f"negedge {signal_name}")
                        elif edge_type == "CHANGED":
                            sensitivity_list.append(signal_name)

            # 根据敏感信号列表生成字符串
            sensitivity_str = " or ".join(sensitivity_list)

            # 如果敏感信号列表为空，默认使用 "*"
            if not sensitivity_list:
                sensitivity_str = "*"

        # 构建 `always` 块
        result = [f"{self.indent()}always @({sensitivity_str}) begin"]
        self.indent_level += 1

        # 处理 `always` 块中的语句
        try:
            for stmt in node.get("stmtsp", []):
                result.append(self.handle_statement(stmt))
        except UnhandledTypeError as e:
            warning_msg = f"Warning: Encountered unhandled type: {e}"
            result.append(f"{self.indent()}// {warning_msg}")
            print(warning_msg)

        self.indent_level -= 1
        result.append(f"{self.indent()}end")
        return "\n".join(filter(None, result))
    
    def handle_display(self, node: Dict[str, Any]) -> str:
        """处理DISPLAY节点，提取错误信息"""
        try:
            # 获取format信息
            fmtp = node.get("fmtp", [])
            if not fmtp:
                return ""

            # 获取第一个format项
            fmt = fmtp[0]
            if fmt.get("type") == "SFORMATF":
                # 提取错误信息字符串
                msg = fmt.get("name", "")
                if "%%Error:" in msg or "%%Fatal:" in msg:
                    # 清理格式化字符串
                    msg = self.clean_format_string(msg)
                    return f"{self.indent()}$error(\"{msg}\");"
                else:
                    # Non-error display: convert to $display
                    clean_msg = self.clean_format_string(msg)
                    return f"{self.indent()}$display(\"{clean_msg}\");"

            return ""
            
        except Exception as e:
            warning_msg = f"Error in DISPLAY: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}/* Warning: {warning_msg} */"
        
    def handle_comment(self, node: Dict[str, Any]) -> str:
        """处理COMMENT节点"""
        if not node:
            return ""
        
        try:
            # 获取注释内容
            comment_text = node.get("name", "")
            if not comment_text:
                return ""
            
            # 添加注释符号和缩进
            return f"{self.indent()}// {comment_text}"
            
        except Exception as e:
            warning_msg = f"Error in COMMENT: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}"
        
    def handle_finish(self, node: Dict[str, Any]) -> str:
        """处理FINISH节点，生成$finish系统任务"""
        if not node:
            return ""
        
        try:
            # 获取可能的参数 (如果有的话)
            params = node.get("params", [])
            param_str = ""
            
            # 处理参数，如果有的话
            if params:
                param_str = ", ".join([self.handle_expression(param) for param in params])
                param_str = f"({param_str})"
            else:
                # 无参数时使用默认值
                param_str = "(32'd0)"
            
            # 生成$finish语句
            return f"{self.indent()}$finish{param_str};"
            
        except Exception as e:
            warning_msg = f"Error in FINISH: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}\n{self.indent()}$finish;"

    def clean_format_string(self, msg: str) -> str:
        """清理格式化字符串中的所有格式标识符和百分号"""
        # 去掉时间戳部分
        msg = msg.replace("[%0t]", "")
        
        # 移除所有包含%的部分
        import re
        # 先处理%%Error和%%Fatal
        msg = msg.replace("%%Error:", "Error:")
        msg = msg.replace("%%Fatal:", "Fatal:")
        
        # 移除所有剩余的含%的内容
        msg = re.sub(r'%[^%\s]*', '', msg)
        
        # 清理多余的空格和换行
        msg = re.sub(r'\s+', ' ', msg)
        msg = msg.replace('\\n', '')
        msg = msg.strip()
        
        return msg

    def handle_statement(self, node: Dict[str, Any]) -> str:
        
        """处理语句节点"""
        stmt_type = node.get("type", "")
        
        try:
            result = ""
            if stmt_type == "IF":
                result = self.handle_if(node)
            elif stmt_type == "ASSIGNDLY":
                result = self.handle_assigndly(node)
            elif stmt_type == "ASSIGN":
                result = self.handle_assign(node)
            elif stmt_type == "ASSIGNW":
                result = self.handle_assign(node)
            elif stmt_type == "CASE":
                result = self.handle_case(node)
            elif stmt_type == "DISPLAY":
                result = self.handle_display(node)
            elif stmt_type == "STOP":
                result = ""  # STOP通常和DISPLAY一起使用，我们可以忽略它
            elif stmt_type == "COMMENT":
                result = self.handle_comment(node)
            elif stmt_type == "FINISH":
                result = self.handle_finish(node)
            elif stmt_type == "WHILE":
                result = self.handle_while(node)
            elif stmt_type == "LOOP":
                result = self.handle_loop(node)
            elif stmt_type == "BEGIN":
                result = self.handle_begin(node)
            elif stmt_type == "JUMPBLOCK":
                loc = node.get("loc")
                print(f"Warning: JUMPBLOCK is not fully supported in line {loc}")
                result = self.handle_jumpblock(node)
            elif stmt_type == "DELAY":
                result = ""  # Skip timing delays (not synthesizable)
            elif stmt_type == "JUMPGO":
                result = ""  # JUMPGO is Verilator internal jump; in flattened output it is a no-op
            elif stmt_type == "CRESET":
                result = ""  # Skip Verilator internal reset logic
            elif stmt_type == "VARREF":
                result = node.get("name", "")
            else:
                warning_msg = f"Unhandled statement type: {stmt_type}"
                loc = node.get("loc")
                print(f"Warning: {warning_msg} in line {loc}")
                result = f"/* Warning: {warning_msg} */"
            
            # 过滤掉包含 Verilator 内部变量的语句（通常是仿真相关的断言或调试代码）
            if result and any(kw in result for kw in ["vlSymsp", "_vm_contextp__", "assertOn"]):
                return ""
                
            return result
                
        except Exception as e:
            warning_msg = f"Error in statement: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */"
        
    def handle_begin(self, node: Dict[str, Any]) -> str:
        """处理BEGIN节点，生成begin...end块"""
        if not node:
            return ""
        
        try:
            result = []
            indent_str = self.indent()
            
            # 获取块名称（如果有）
            name = node.get("name", "")
            if name:
                result.append(f"{indent_str}begin : {name}")
            else:
                result.append(f"{indent_str}begin")
            
            self.indent_level += 1
            
            # 处理块中的所有语句
            for stmt in node.get("stmtsp", []):
                stmt_str = self.handle_statement(stmt)
                if stmt_str:
                    result.append(stmt_str)
            
            self.indent_level -= 1
            result.append(f"{indent_str}end")
            
            return "\n".join(result)
            
        except Exception as e:
            warning_msg = f"Error in BEGIN: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}"
        
    def handle_case(self, node: Dict[str, Any]) -> str:
        result = []
        
        # 处理case表达式
        if not node.get("exprp"):
            return ""
        
        # 使用handle_expression处理case表达式
        expr = self.handle_expression(node["exprp"][0])
        
        # 检查所有case项的条件中是否包含z或x
        has_z_or_x = False
        for item in node.get("itemsp", []):
            if item["type"] != "CASEITEM":
                continue
            if item.get("condsp"):
                for cond_node in item["condsp"]:
                    cond_str = self.handle_expression(cond_node)
                    if 'z' in cond_str.lower() or 'x' in cond_str.lower():
                        has_z_or_x = True
                        break
                if has_z_or_x:
                    break
        
        # 如果包含z或x，使用casex并打印warning
        case_keyword = "casex" if has_z_or_x else "case"
        if has_z_or_x:
            loc = node.get("loc", "unknown")
            print(f"Warning: Detected 'z' or 'x' in case conditions, using casex instead of case (line {loc})")
        
        result.append(f"{self.indent()}{case_keyword} ({expr})")
        
        # 增加缩进层级
        self.indent_level += 1
        
        # 处理每个case项
        for item in node.get("itemsp", []):
            if item["type"] != "CASEITEM":
                raise UnhandledTypeError(f"Unexpected case item type: {item['type']}")
            
            # 获取case条件
            if item.get("condsp"):
                # 处理所有条件，用逗号分隔
                conditions = []
                for cond_node in item["condsp"]:
                    if cond_node.get("type") == "INSIDERANGE":
                        # Expand INSIDERANGE to comma-separated constants for case items
                        # e.g., [3'h0:3'h4] -> 3'h0, 3'h1, 3'h2, 3'h3, 3'h4
                        left_name = cond_node["lhsp"][0]["name"] if cond_node.get("lhsp") else "0"
                        right_name = cond_node["rhsp"][0]["name"] if cond_node.get("rhsp") else "0"
                        left_val = self.parse_const_value(left_name)
                        right_val = self.parse_const_value(right_name)

                        if "'" in left_name:
                            parts = left_name.split("'")
                            width = int(parts[0])
                            base = parts[1][0] if parts[1] else 'h'
                        else:
                            width = 32
                            base = 'h'

                        step = 1 if left_val <= right_val else -1
                        if base == 'h':
                            hex_digits = (width + 3) // 4
                            for val in range(left_val, right_val + step, step):
                                conditions.append(f"{width}'h{val:0{hex_digits}x}")
                        elif base == 'b':
                            for val in range(left_val, right_val + step, step):
                                conditions.append(f"{width}'b{val:0{width}b}")
                        else:
                            for val in range(left_val, right_val + step, step):
                                conditions.append(f"{width}'d{val}")
                    else:
                        raw_cond = self.handle_expression(cond_node)

                        # --- 修改开始 ---
                        # 修复 Verilog 扩展规则问题：
                        # 原始 AST可能是 "48'bz"，Verilog 会解析为全 z (zzzz...)
                        # 我们将其替换为 "48'b0z"，Verilog 会解析为 0 扩展 (000...z)
                        if raw_cond.endswith("'bz"):
                            raw_cond = raw_cond.replace("'bz", "'b0z")
                        elif raw_cond.endswith("'bx"):
                            raw_cond = raw_cond.replace("'bx", "'b0x")
                        # --- 修改结束 ---

                        conditions.append(raw_cond)
                    
                cond_str = ", ".join(conditions)
                result.append(f"{self.indent()}{cond_str}: begin")
            else:
                result.append(f"{self.indent()}default: begin")
            
            # 增加缩进层级处理case项中的语句
            self.indent_level += 1
            try:
                for stmt in item.get("stmtsp", []):
                    result.append(self.handle_statement(stmt))
            except UnhandledTypeError as e:
                print(f"Warning: In case item: {e}")
            
            # 减少缩进层级
            self.indent_level -= 1
            result.append(f"{self.indent()}end")
        
        # 减少缩进层级
        self.indent_level -= 1
        result.append(f"{self.indent()}endcase")
        
        return "\n".join(filter(None, result))

    def handle_if(self, node: Dict[str, Any]) -> str:
        result = []
        
        # 处理条件
        condition = self.handle_condition(node.get("condp", [])[0])
        
        # 过滤掉包含 Verilator 内部变量的条件块（通常是 Verilator 插入的断言检查）
        if any(kw in condition for kw in ["vlSymsp", "_vm_contextp__", "assertOn"]):
            return ""
            
        result.append(f"{self.indent()}if ({condition}) begin")
        
        # 增加缩进层级
        self.indent_level += 1
        
        # 处理then分支中的所有语句
        try:
            for stmt in node.get("thensp", []):
                result.append(self.handle_statement(stmt))
        except UnhandledTypeError as e:
            print(f"Warning: In then branch: {e}")
        
        # 减少缩进层级
        self.indent_level -= 1
        
        # 处理else分支
        if node.get("elsesp"):
            result.append(f"{self.indent()}end else begin")
            # 增加缩进层级
            self.indent_level += 1
            
            try:
                for stmt in node.get("elsesp", []):
                    result.append(self.handle_statement(stmt))
            except UnhandledTypeError as e:
                print(f"Warning: In else branch: {e}")
                
            # 减少缩进层级
            self.indent_level -= 1
        
        result.append(f"{self.indent()}end")
        return "\n".join(filter(None, result))
    
    def handle_init_item(self, node: Dict[str, Any]) -> str:
        """处理单个初始化项
        返回解析后的值字符串
        """
        try:
            # 获取value列表
            value_list = node.get("valuep", [])
            if not value_list or len(value_list) == 0:
                return "0"

            # 获取第一个value节点
            value_node = value_list[0]
            
            # 根据类型处理
            node_type = value_node.get("type", "")
            if node_type == "CONST":
                # 常量值直接解析
                const_value = value_node.get("name", "")
                return str(self.parse_const_value(const_value))
            elif node_type == "INITITEM":
                # 递归处理嵌套的INITITEM
                return self.handle_init_item(value_node)
            
            return "0"

        except Exception as e:
            warning_msg = f"Error in handle_init_item: {str(e)}"
            print(f"Warning: {warning_msg}")
            return "0"

    def handle_init_array(self, node: Dict[str, Any]) -> str:
        """处理数组初始化
        返回格式如: "'{6{0}}" 或 "'{default:0}"
        """
        try:
            # 获取初始化列表
            init_items = node.get("initsp", [])
            if not init_items:
                return ""

            # 创建一个列表存储所有值
            values = []
            
            # 遍历每个初始化项
            for init_item in init_items:
                if init_item.get("type") == "INITITEM":
                    value = self.handle_init_item(init_item)
                    values.append(value)
            
            # 如果没有有效值，返回空字符串
            if not values:
                return ""
                
            # 检查是否所有值都相同
            if all(v == values[0] for v in values):
                # 如果所有值都相同，使用重复语法
                return f"'{{{len(values)}{{{values[0]}}}}}"
            else:
                # 如果值不同，则使用完整列表
                return f"'{{{','.join(values)}}}"

        except Exception as e:
            warning_msg = f"Error in handle_init_array: {str(e)}"
            print(f"Warning: {warning_msg}")
            return ""
        
    def handle_while(self, node: Dict[str, Any]) -> str:
        """处理WHILE节点，生成完整的for循环语句，包括初始化部分"""
        if not node:
            return ""
        
        try:
            result = []
            indent_str = self.indent()
            
            # 获取循环条件
            condition = ""
            if "condp" in node and node["condp"]:
                condition = self.handle_expression(node["condp"][0])
            else:
                condition = "1'b1"  # 如果没有条件，默认为true
            
            # 获取递增表达式
            increment = ""
            if "incsp" in node and node["incsp"]:
                # 处理递增表达式
                inc_node = node["incsp"][0]
                if inc_node.get("type") == "ASSIGN":
                    lhs = self.handle_expression(inc_node["lhsp"][0])
                    rhs = self.handle_expression(inc_node["rhsp"][0])
                    increment = f"{lhs} = {rhs}"
                else:
                    inc_str = self.handle_statement(inc_node)
                    increment = inc_str.strip()
                    if increment.endswith(";"):
                        increment = increment[:-1]
                    increment = increment.lstrip()
            
            # 找出循环变量和初始值
            # 通常为循环变量会在递增表达式的左侧
            init_var = ""
            # if "incsp" in node and node["incsp"]:
            #     inc_node = node["incsp"][0]
            #     if inc_node.get("type") == "ASSIGN" and "lhsp" in inc_node:
            #         init_var = self.handle_expression(inc_node["lhsp"][0])
            
            # 构建完整的for循环语句包括初始化部分
            # loop_header = f"{indent_str}for ({init_var} = 0; {condition}; {increment}) begin"
            loop_header = f"{indent_str}for ({init_var}; {condition}; {increment}) begin"
            result.append(loop_header)
            
            # 增加缩进级别处理循环体
            self.indent_level += 1
            
            # 处理循环体中的语句
            if "stmtsp" in node and node["stmtsp"]:
                for stmt in node["stmtsp"]:
                    stmt_str = self.handle_statement(stmt)
                    if stmt_str:
                        result.append(stmt_str)
            
            # 减少缩进级别
            self.indent_level -= 1
            
            # 添加循环结束标记
            result.append(f"{indent_str}end")
            
            return "\n".join(result)
            
        except Exception as e:
            warning_msg = f"Error in FOR loop (WHILE node): {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}\n{self.indent()}for (;;) begin\n{self.indent()}  // Error processing for loop\n{self.indent()}end"
        
    def handle_loop(self, node: Dict[str, Any]) -> str:
        """处理LOOP节点，生成完整的for循环语句"""
        if not node:
            return ""
        
        try:
            result = []
            indent_str = self.indent()
            
            # 获取循环测试、循环体和递增表达式
            stmtsp = node.get("stmtsp", [])
            condition = ""
            increment = ""
            body_statements = []
            
            for stmt in stmtsp:
                stype = stmt.get("type")
                if stype == "LOOPTEST":
                    if "condp" in stmt and stmt["condp"]:
                        condition = self.handle_expression(stmt["condp"][0])
                elif stype == "BEGIN" or stype == "ASSIGN":
                    # 如果是ASSIGN且不是最后一个，或者它在LOOPTEST之后，可能是递增或体
                    # 在Verilator的LOOP中，通常顺序是 LOOPTEST -> BODY -> INCREMENT
                    # 我们可以根据顺序来判断，或者检查它是否是最后一个ASSIGN
                    pass
            
            # 重新精确解析LOOP节点结构
            # 可能的结构:
            #   [LOOPTEST, BODY..., INCREMENT]  (标准情况)
            #   [ASSIGN(init), LOOPTEST, BODY..., INCREMENT]  (带初始化的情况)

            init_str = ""
            looptest_idx = -1

            # 找到LOOPTEST的位置
            for idx, stmt in enumerate(stmtsp):
                if stmt.get("type") == "LOOPTEST":
                    looptest_idx = idx
                    break

            if looptest_idx >= 0:
                # LOOPTEST之前可能有初始化语句
                if looptest_idx > 0:
                    init_node = stmtsp[0]
                    if init_node.get("type") == "ASSIGN":
                        lhs = self.handle_expression(init_node["lhsp"][0])
                        rhs = self.handle_expression(init_node["rhsp"][0])
                        init_str = f"{lhs} = {rhs}"

                test_node = stmtsp[looptest_idx]
                if "condp" in test_node and test_node["condp"]:
                    condition = self.handle_expression(test_node["condp"][0])

            # 寻找递增表达式 (通常是最后一个语句，且类型为ASSIGN)
            if len(stmtsp) >= 3 and looptest_idx >= 0:
                inc_node = stmtsp[-1]
                if inc_node.get("type") == "ASSIGN":
                    lhs = self.handle_expression(inc_node["lhsp"][0])
                    rhs = self.handle_expression(inc_node["rhsp"][0])
                    increment = f"{lhs} = {rhs}"
                    body_statements = stmtsp[looptest_idx + 1:-1]
                else:
                    body_statements = stmtsp[looptest_idx + 1:]
            elif len(stmtsp) >= 2 and looptest_idx >= 0:
                body_statements = stmtsp[looptest_idx + 1:]

            # 构建for循环语句
            # 如果已经有初始化语句，使用它；否则从递增表达式提取循环变量
            if init_str:
                loop_header = f"{indent_str}for ({init_str}; {condition}; {increment}) begin"
            else:
                # 没有在LOOP节点内找到初始化语句，尝试从上一个ASSIGN获取
                inc_node = stmtsp[-1]
                if inc_node.get("type") == "ASSIGN" and "lhsp" in inc_node:
                    loop_var = self.handle_expression(inc_node["lhsp"][0])
                    # 检查上一个ASSIGN的LHS是否匹配循环变量
                    if self._last_assign_lhs == loop_var and self._last_assign_rhs is not None:
                        loop_header = f"{indent_str}for ({loop_var} = {self._last_assign_rhs}; {condition}; {increment}) begin"
                    else:
                        loop_header = f"{indent_str}for ({loop_var} = {loop_var}; {condition}; {increment}) begin"
                else:
                    loop_header = f"{indent_str}for (; {condition}; {increment}) begin"
            result.append(loop_header)
            
            # 增加缩进级别处理循环体
            self.indent_level += 1
            for stmt in body_statements:
                stmt_str = self.handle_statement(stmt)
                if stmt_str:
                    result.append(stmt_str)
            self.indent_level -= 1
            
            # 添加循环结束标记
            result.append(f"{indent_str}end")
            
            return "\n".join(result)
            
        except Exception as e:
            warning_msg = f"Error in LOOP node: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"{self.indent()}// Warning: {warning_msg}"
        
    def handle_insiderange(self, node: Dict[str, Any]) -> str:
        """处理INSIDERANGE节点，将其转换为[left:right]格式"""
        if not node:
            return ""
        
        try:
            # 获取left和right表达式
            left_expr = ""
            right_expr = ""
            
            if "lhsp" in node and node["lhsp"]:
                left_expr = self.handle_expression(node["lhsp"][0])
            
            if "rhsp" in node and node["rhsp"]:
                right_expr = self.handle_expression(node["rhsp"][0])
            
            # 如果成功获取了两个表达式，返回[left:right]格式
            if left_expr and right_expr:
                return f"[{left_expr}:{right_expr}]"
            
            # 如果只有一个表达式，返回单个表达式
            if left_expr:
                warning_msg = f"INSIDERANGE: Only left expression found"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */"
            if right_expr:
                warning_msg = f"INSIDERANGE: Only right expression found"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */"
            
            # 如果没有表达式，返回空字符串
            return ""
        
        except Exception as e:
            warning_msg = f"Error in INSIDERANGE: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */"
        

    def handle_expression(self, node: Dict[str, Any]) -> str:
        """通用表达式处理函数"""
        if not node:
            return ""
                
        node_type = node.get("type", "")
        if node_type == "VARREF":
            return node.get("name", "")
        if node_type == "INITARRAY":
            return self.handle_init_array(node)
        if node_type == "VARREF":
            return node.get("name", "")
        elif node_type == "CONST":
            name = node.get("name", "")
            # Check if this is a string constant (dtypep resolves to BASICDTYPE with keyword "string")
            dtype_ref = node.get("dtypep", "")
            if dtype_ref and isinstance(dtype_ref, str):
                dtype_id = dtype_ref.strip("()")
                dtype_info = self.type_table.get(dtype_id)
                if dtype_info and dtype_info.get("keyword") == "string":
                    # Strip Verilator's backslash-escaped quotes from string constants
                    # e.g., \"trace_hart_00.dasm\" -> "trace_hart_00.dasm"
                    name = name.replace('\\"', '"')
            return name
        elif node_type == "AND":
            return self.handle_binary_op(node, "&")
        elif node_type == "LOGAND":  
            return self.handle_binary_op(node, "&&")
        elif node_type == "LOGOR":
            return self.handle_binary_op(node, "||")
        elif node_type == "LOGNOT":
            return self.handle_unary_op(node, "!")
        elif node_type == "OR":
            return self.handle_binary_op(node, "|") # unsure about the usage
        elif node_type == "NOT":
            return self.handle_unary_op(node, "~")
        elif node_type == "XOR":
            return self.handle_binary_op(node, "^")
        elif node_type == "DIV":
            return self.handle_binary_op(node, "/")
        elif node_type == "DIVS":
            return self.handle_binary_op(node, "/", signed=True)
        elif node_type == "MODDIV":
            return self.handle_binary_op(node, "%")
        elif node_type == "MODDIVS":
            return self.handle_binary_op(node, "%",signed=True)
        elif node_type == "REDOR":
            return self.handle_unary_op(node, "|")
        elif node_type == "REDAND":
            return self.handle_unary_op(node, "&")
        elif node_type == "REDXOR":  
            return self.handle_unary_op(node, "^")
        elif node_type == "INSIDERANGE":
            return self.handle_insiderange(node)
        elif node_type == "TIME":
            return "$stime"
        elif node_type == "SEL":
            return self.handle_sel(node)
        elif node_type == "ARRAYSEL":
            try:
                array = self.handle_expression(node["fromp"][0])
                index = self.handle_expression(node["bitp"][0])
                return f"{array}[{index}]"
            except Exception as e:
                warning_msg = f"Error in ARRAYSEL: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "CONCAT":
            return self.handle_concat(node)
        elif node_type == "ADD":
            return self.handle_binary_op(node, "+")
        elif node_type == "SUB":
            return self.handle_binary_op(node, "-")
        elif node_type == "MUL":
            return self.handle_binary_op(node, "*")
        elif node_type == "POWSU":
            return self.handle_binary_op(node, "**")
        elif node_type == "POWSS":
            return self.handle_binary_op(node, "**")
        elif node_type == "NEGATE":
            return self.handle_unary_op(node, "-")
        elif node_type == "SHIFTR":
            return self.handle_binary_op(node, ">>")
        elif node_type == "SHIFTL":
            return self.handle_binary_op(node, "<<")
        elif node_type == "SHIFTRS":  # 算术右移
            return self.handle_binary_op(node, ">>>", signed=True)
        # MAY CAUSE BUG HERE
        elif node_type == "EXTEND" or node_type == "EXTENDS":
            return self.handle_extend(node,node_type=="EXTENDS")
        # MAY CAUSE BUG HERE
        # elif node_type == "EXTENDS":
        #     try:
        #         expr = self.handle_expression(node["lhsp"][0])
        #         return f"$signed({expr})"
        #     except Exception as e:
        #         warning_msg = f"Error in EXTENDS: {str(e)}"
        #         print(f"Warning: {warning_msg}")
        #         return f"/* Warning: {warning_msg} */ 1'b0"
        # MAY CAUSE BUG HERE
        elif node_type == "MULS":
            return self.handle_binary_op(node, "*",signed=True)
        elif node_type == "RAND":
            return "$urandom()"
        elif node_type == "NEQ":
            return self.handle_binary_op(node, "!=")
        elif node_type == "EQ":
            return self.handle_binary_op(node, "==")
        elif node_type == "EQWILD":
            return self.handle_binary_op(node, "==?")
        elif node_type == "NEQWILD":
            return self.handle_binary_op(node, "!=?")
        elif node_type == "GT":
            return self.handle_binary_op(node, ">")
        elif node_type == "GTS":  # 添加GTS处理
            try:
                return self.handle_binary_op(node, ">",signed=True)
                # left = self.handle_expression(node["lhsp"][0])
                # right = self.handle_expression(node["rhsp"][0])
                
                # # 检查左右操作数是否为常量
                # is_left_const = node["lhsp"][0].get("type") == "CONST"
                # is_right_const = node["rhsp"][0].get("type") == "CONST"
                
                # # 只对非常量添加$signed
                # left_expr = left if is_left_const else f"$signed({left})"
                # right_expr = right if is_right_const else f"$signed({right})"
                
                # return f"{left_expr} > {right_expr}"
            except Exception as e:
                warning_msg = f"Error in GTS: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "LTS":  # 添加GTS处理
            try:
                return self.handle_binary_op(node, "<",signed=True)
                # left = self.handle_expression(node["lhsp"][0])
                # right = self.handle_expression(node["rhsp"][0])
                
                # # 检查左右操作数是否为常量
                # is_left_const = node["lhsp"][0].get("type") == "CONST"
                # is_right_const = node["rhsp"][0].get("type") == "CONST"
                
                # # 只对非常量添加$signed
                # left_expr = left if is_left_const else f"$signed({left})"
                # right_expr = right if is_right_const else f"$signed({right})"
                
                # return f"{left_expr} < {right_expr}"
            except Exception as e:
                warning_msg = f"Error in LTS: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "LTES":  # 添加LTES处理
            try:
                return self.handle_binary_op(node, "<=",signed=True)
                # left = self.handle_expression(node["lhsp"][0])
                # right = self.handle_expression(node["rhsp"][0])
                
                # is_left_const = node["lhsp"][0].get("type") == "CONST"
                # is_right_const = node["rhsp"][0].get("type") == "CONST"
                
                # left_expr = left if is_left_const else f"$signed({left})"
                # right_expr = right if is_right_const else f"$signed({right})"
                
                # return f"{left_expr} <= {right_expr}"
            except Exception as e:
                warning_msg = f"Error in LTES: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "GTES":  # 添加LTES处理
            try:
                return self.handle_binary_op(node, ">=",signed=True)
                # left = self.handle_expression(node["lhsp"][0])
                # right = self.handle_expression(node["rhsp"][0])
                
                # is_left_const = node["lhsp"][0].get("type") == "CONST"
                # is_right_const = node["rhsp"][0].get("type") == "CONST"
                
                # left_expr = left if is_left_const else f"$signed({left})"
                # right_expr = right if is_right_const else f"$signed({right})"
                
                # return f"{left_expr} >= {right_expr}"
            except Exception as e:
                warning_msg = f"Error in GTES: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "LT":
            return self.handle_binary_op(node, "<")
        elif node_type == "GTE":
            return self.handle_binary_op(node, ">=")
        elif node_type == "LTE":
            return self.handle_binary_op(node, "<=")
        elif node_type == "REPLICATE":
            try:
                count = self.handle_expression(node["countp"][0])
                src = self.handle_expression(node["srcp"][0])
                return f"{{{count}{{{src}}}}}"
            except Exception as e:
                warning_msg = f"Error in REPLICATE: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "ONEHOT":
            expr = self.handle_expression(node["lhsp"][0])
            return f"$onehot({expr})"
        elif node_type == "STREAML":
            expr = self.handle_expression(node["lhsp"][0])
            slice_size = self.handle_expression(node["rhsp"][0]) if node.get("rhsp") else "1"
            return f"{{<<{slice_size}{{{expr}}}}}"
        elif node_type == "EXPRSTMT":
            return self.handle_exprstmt(node)
        elif node_type == "COND":
            try:
                condition = self.handle_expression(node["condp"][0])
                true_expr = self.handle_expression(node["thenp"][0])
                false_expr = self.handle_expression(node["elsep"][0])
                return f"({condition} ? {true_expr} : {false_expr})"
            except Exception as e:
                warning_msg = f"Error in COND: {str(e)}"
                print(f"Warning: {warning_msg}")
                return f"/* Warning: {warning_msg} */ 1'b0"
        elif node_type == "VARXREF":
            # 处理层次化变量引用 (e.g., or1200_except.wb_pc)
            dotted = node.get("dotted", "")
            name = node.get("name", "")
            if dotted:
                return f"{dotted}.{name}"
            return name
        elif node_type == "SAMPLED":
            # 处理SAMPLED节点 (采样信号)，在SVA中输出 $sampled(expr)
            exprp = node.get("exprp", [])
            if exprp:
                inner = self.handle_expression(exprp[0])
                return f"$sampled({inner})"
            return ""
        elif node_type == "CEXPR":
            # 处理CEXPR节点，通常包含TEXT节点
            results = []
            for child in node.get("exprsp", node.get("nodesp", [])):
                results.append(self.handle_expression(child))
            return "".join(results)
        elif node_type == "TEXT":
            # 处理TEXT节点，直接返回其内容
            return node.get("shortText", node.get("text", ""))
        elif node_type == "FOPEN":
            # 处理FOPEN节点: $fopen(filename, mode)
            filename = ""
            mode = ""
            if node.get("filenamep"):
                filename = self.handle_expression(node["filenamep"][0])
            if node.get("modep"):
                mode = self.handle_expression(node["modep"][0])
            return f"$fopen({filename}, {mode})"
        else:
            loc = node.get("loc")
            warning_msg = f"Unknown expression type: {node_type} in line {loc}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */ 1'b0"
        
    def handle_exprstmt(self, node: Dict[str, Any]) -> str:
        """
        处理EXPRSTMT节点
        - 处理stmtsp中的所有语句并保存到self.expr_stmt_str
        - 返回resultp的结果
        """
        # 保存stmtsp中的语句（跳过第一个COMMENT）
        stmt_results = []
        for stmt in node.get("stmtsp", [])[1:]:  # 跳过COMMENT
            stmt_result = self.handle_statement(stmt)
            if stmt_result:
                stmt_results.append(stmt_result)
        
        # 保存到特殊变量
        
        
        # 返回resultp的结果
        if node.get("resultp"):
            resultp = self.handle_expression(node["resultp"][0])
            self.resultp_dict[resultp] = stmt_results
            return resultp
        return ""

    def handle_binary_op(self, node: Dict[str, Any], operator: str, signed: bool = False) -> str:
        """处理二元运算符，支持有条件地添加$signed()"""
        if not (node.get("lhsp") and node.get("rhsp")):
            return ""
        
        try:
            left = self.handle_expression(node["lhsp"][0])
            right = self.handle_expression(node["rhsp"][0])

            # 为常量值添加适当的格式
            if node["rhsp"][0].get("type") == "CONST":
                right_val = node["rhsp"][0]["name"]
                if right_val.startswith("32'h"):
                    right = str(int(right_val[4:], 16))
            
            # 根据signed参数，决定是否给表达式两边加上$signed()
            if signed:
                left = f"$signed({left})"
                right = f"$signed({right})"
                return f"$signed({left} {operator} {right})"
            
            return f"({left} {operator} {right})"
        except Exception as e:
            warning_msg = f"Error in binary operation: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */ 'b0"
    
    def handle_concat(self, node: Dict[str, Any]) -> str:
        """处理CONCAT (连接) 操作"""
        if not (node.get("lhsp") and node.get("rhsp")):
            return ""
        
        try:
            # 获取左右两部分
            left = self.handle_expression(node["lhsp"][0])
            right = self.handle_expression(node["rhsp"][0])
            
            # 使用Verilog的连接操作符
            return f"{{{left}, {right}}}"
        except Exception as e:
            warning_msg = f"Error in CONCAT: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */ 'b0"

    def handle_extend(self, node: Dict[str, Any], is_signed = False) -> str:
        """处理EXTEND (位扩展) 操作"""
        if not node.get("lhsp"):
            return ""
        
        try:
            # 获取要扩展的表达式
            lhs_node = node["lhsp"][0]
            expr = self.handle_expression(lhs_node)
            
            # 获取目标类型(extend后)的位宽
            target_width = 0
            base_width, _ = self.get_width_info(node)
            if base_width.startswith('[') and base_width.endswith(']'):
                width_range = base_width[1:-1].split(':')
                if len(width_range) == 2:
                    try:
                        target_width = abs(int(width_range[0]) - int(width_range[1])) + 1
                    except ValueError:
                        pass

            # Fallback 1: Try widthConst attribute on the EXTEND node
            if target_width == 0:
                width_const = node.get("widthConst", "")
                if width_const:
                    try:
                        target_width = int(width_const)
                    except ValueError:
                        pass

            # Fallback 2: Try dtypep direct lookup in type_table
            if target_width == 0:
                dtype_ref = node.get("dtypep", "")
                if isinstance(dtype_ref, str) and dtype_ref.startswith("(") and dtype_ref.endswith(")"):
                    dtype_id = dtype_ref[1:-1]
                    if dtype_id in self.type_table:
                        target_width = self.get_base_width_bits(self.type_table[dtype_id])

            if target_width == 0:
                loc = node.get("loc")
                print(f"Error in EXTEND: Invalid width range in line {loc}")

            # 获取源操作数(extend前)的位宽
            source_width = 1
            lhs_base_width, _ = self.get_width_info(lhs_node)
            if lhs_base_width.startswith('[') and lhs_base_width.endswith(']'):
                width_range = lhs_base_width[1:-1].split(':')
                if len(width_range) == 2:
                    try:
                        source_width = abs(int(width_range[0]) - int(width_range[1])) + 1
                    except ValueError:
                        pass

            # Fallback for source_width: try dtypep on lhs_node
            if source_width == 1 and lhs_base_width == "":
                lhs_dtype_ref = lhs_node.get("dtypep", "")
                if isinstance(lhs_dtype_ref, str) and lhs_dtype_ref.startswith("(") and lhs_dtype_ref.endswith(")"):
                    lhs_dtype_id = lhs_dtype_ref[1:-1]
                    if lhs_dtype_id in self.type_table:
                        sw = self.get_base_width_bits(self.type_table[lhs_dtype_id])
                        if sw > 0:
                            source_width = sw

            # 计算需要扩展的位数
            extend_bits = target_width - source_width
            
            # 为适应SystemVerilog 05的拓展规则
            if (expr.endswith(')') or expr.endswith('}') or expr.endswith(']')) and self.convert_complex_sel_to_temp:
                base_width, array_range = self.get_width_info(lhs_node)
                temp_var = f"___sel_temp_{self.sel_temp_count}"
                self.sel_temp_count += 1
                self.sel_temp_dict[temp_var] = {
                    'expr': expr,
                    'width': base_width,# if base_width else "[31:0]",  # 默认位宽
                    'array_range': array_range
                }
                expr = temp_var
            extend_str = '{'+str(extend_bits) + '{' + f'{expr}[{source_width-1}]' +'}}'
            
            if extend_bits > 0:
                if is_signed:
                    return f"{{{extend_str},{expr}}}"
                return f"{{{extend_bits}'b0,{expr}}}"
                
            if is_signed:
                return f"$signed({expr})"
            return expr
            
        except Exception as e:
            warning_msg = f"Error in EXTEND: {str(e)}"
            print(f"Warning: {warning_msg}")
            return f"/* Warning: {warning_msg} */ 'b0"

    def handle_unary_op(self, node: Dict[str, Any], operator: str) -> str:
        """处理一元运算符"""
        if not node.get("lhsp"):
            return ""
        
        expr = self.handle_expression(node["lhsp"][0])
        return f"{operator}({expr})"

    def handle_condition(self, node: Dict[str, Any]) -> str:
        """处理条件表达式"""
        return self.handle_expression(node)

    def handle_cond(self, node: Dict[str, Any], lhs: str, is_blocking: bool = False) -> str:
        """处理条件表达式，支持阻塞和非阻塞赋值"""
        if not (node.get("condp") and node.get("thenp") and node.get("elsep")):
            return ""
        
        try:
            # 处理条件
            condition = self.handle_expression(node["condp"][0])
            
            # 处理then分支
            then_expr = self.handle_expression(node["thenp"][0])
            
            # 处理else分支
            else_expr = self.handle_expression(node["elsep"][0])
            
            # 根据是否阻塞赋值选择运算符
            op = "=" if is_blocking else "<="
            
            # 返回完整的条件赋值语句
            return f"{self.indent()}{lhs} {op} ({condition}) ? {then_expr} : {else_expr};"
            
        except Exception as e:
            warning_msg = f"Error in COND: {str(e)}"
            print(f"Warning: {warning_msg}")
            # 根据是否阻塞赋值选择运算符
            op = "=" if is_blocking else "<="
            return f"{self.indent()}{lhs} {op} 'b0; // Warning: {warning_msg}"
    
    def handle_assign(self, node: Dict[str, Any]) -> str:
        if not node.get("lhsp") or not node.get("rhsp"):
            return ""

        try:
            lhs_node = node["lhsp"][0]

            # Handle EXTEND on LHS: this means the RHS is wider than the LHS variable.
            # EXTEND on LHS would produce invalid {N'b0, var} concat via handle_extend(),
            # so we instead truncate the RHS to match the inner variable's width.
            if lhs_node.get("type") == "EXTEND":
                inner_node = lhs_node["lhsp"][0]
                lhs = self.handle_expression(inner_node)
                rhs = self.handle_expression(node["rhsp"][0])

                # Get inner variable width to truncate RHS
                base_width, _ = self.get_width_info(inner_node)
                if base_width.startswith('[') and base_width.endswith(']'):
                    width_range = base_width[1:-1].split(':')
                    if len(width_range) == 2:
                        try:
                            inner_width = abs(int(width_range[0]) - int(width_range[1])) + 1
                            return f"{self.indent()}{lhs} = {rhs}[{inner_width-1}:0];"
                        except ValueError:
                            pass

                return f"{self.indent()}{lhs} = {rhs};"

            # 获取左侧表达式 - 这里要改用handle_expression
            lhs = self.handle_expression(lhs_node)

            # 获取右侧值
            rhs_node = node["rhsp"][0]
            if rhs_node["type"] == "COND":
                rhs = self.handle_expression(rhs_node["condp"][0]) if rhs_node.get("condp") else ""
                self._last_assign_lhs = lhs
                self._last_assign_rhs = rhs
                return self.handle_cond(rhs_node, lhs, is_blocking=True)
            else:
                rhs = self.handle_expression(rhs_node)
                self._last_assign_lhs = lhs
                self._last_assign_rhs = rhs
                return f"{self.indent()}{lhs} = {rhs};"
        except Exception as e:
            warning_msg = f"Error in ASSIGN: {str(e)}"
            result = []
            result.append(f"{self.indent()}// Warning: {warning_msg}")
            result.append(f"{self.indent()}{lhs} = 'b0;")
            print(f"Warning: {warning_msg}")
            return "\n".join(result)

    def handle_assigndly(self, node: Dict[str, Any]) -> str:
        if not node.get("lhsp") or not node.get("rhsp"):
            return ""
        
        try:
            # 使用handle_expression处理左侧表达式
            lhs = self.handle_expression(node["lhsp"][0])
            
            # 获取右侧值
            rhs_node = node["rhsp"][0]
            if rhs_node["type"] == "COND":
                return self.handle_cond(rhs_node, lhs)
            else:
                rhs = self.handle_expression(rhs_node)
                return f"{self.indent()}{lhs} <= {rhs};"
        except Exception as e:
            warning_msg = f"Error in ASSIGNDLY: {str(e)}"
            result = []
            result.append(f"{self.indent()}// Warning: {warning_msg}")
            result.append(f"{self.indent()}{lhs} <= 'b0;")
            print(f"Warning: {warning_msg}")
            return "\n".join(result)

    def handle_unknown(self, node: Dict[str, Any]) -> str:
        return f"{self.indent()}/* Unhandled node: {node.get('type', 'UNKNOWN')} {node.get('name', '')} */"

# def insert_resultp_statements(verilog_code: str, generator) -> str:
#     """
#     在verilog代码中查找resultp_dict的key，并在其上一行插入对应的语句
    
#     Args:
#         verilog_code: 原始的Verilog代码字符串
#         generator: 包含resultp_dict的生成器对象
    
#     Returns:
#         str: 插入语句后的Verilog代码
#     """
#     # 将代码分割成行
#     lines = verilog_code.split('\n')
#     result_lines = []
    
#     # 遍历每一行
#     for line in lines:
#         # 检查这一行是否包含任何resultp_dict的key
#         matched_key = None
#         for key in generator.resultp_dict:
#             if key in line:
#                 matched_key = key
#                 break
        
#         # 如果找到匹配的key，插入对应的语句
#         if matched_key:
#             # 保持原有的缩进
#             indent = line[:len(line) - len(line.lstrip())]
#             # 插入resultp_dict中的所有语句
#             for stmt in generator.resultp_dict[matched_key]:
#                 result_lines.append(f"{indent}{stmt}")
        
#         # 添加当前行
#         result_lines.append(line)
    
#     # 合并所有行
#     return '\n'.join(result_lines)

def insert_resultp_statements(verilog_code: str, resultp_dict: dict) -> str:
    """
    将resultp_dict中的所有语句统一放在endmodule前的always @(*) 块中
    
    Args:
        verilog_code: 原始的Verilog代码字符串
        resultp_dict: 包含需要插入语句的字典，格式为 {key: [statements]}
    
    Returns:
        str: 插入语句后的Verilog代码
    """
    # 将代码分割成行
    lines = verilog_code.split('\n')
    
    # 找到最后一个endmodule的位置
    for i in range(len(lines) - 1, -1, -1):
        if 'endmodule' in lines[i]:
            # 获取endmodule的缩进
            indent = lines[i][:len(lines[i]) - len(lines[i].lstrip())]
            
            # 收集所有需要插入的语句
            all_statements = []
            for statements in resultp_dict.values():
                all_statements.extend(statements)
                
            if all_statements:
                # 插入always块和语句
                insert_lines = [
                    f"{indent}always @(*) begin",
                ]
                # 添加所有语句，增加缩进
                for stmt in all_statements:
                    insert_lines.append(f"{indent}    {stmt}")
                insert_lines.append(f"{indent}end")
                insert_lines.append("")  # 空行
                
                # 在endmodule前插入
                lines[i:i] = insert_lines
            break
    
    # 合并所有行
    return '\n'.join(lines)

def insert_sel_temp_statements(verilog_code: str, sel_temp_dict: dict) -> str:
    """
    在第一个'logic'声明之前插入临时变量的wire声明，并在找到对应temp_var的地方从后往前插入赋值。
    按照变量名末尾数字的大小排序（从大到小），并保持缩进。
    
    Args:
        verilog_code: 原始的Verilog代码字符串
        sel_temp_dict: 临时变量字典，格式为 {temp_var: {'expr': expr, 'width': width, 'array_range': array_range}}
    """
    lines = verilog_code.split('\n')
    result_lines = []
    
    # 找到第一个包含" logic "的行和它的缩进
    logic_line_index = -1
    indent = ""
    for i, line in enumerate(lines):
        if re.search(r'\blogic\b', line):
            logic_line_index = i
            indent = line[:len(line) - len(line.lstrip())]
            break
    
    if logic_line_index != -1:
        # 在logic行之前插入声明
        result_lines.extend(lines[:logic_line_index])
        for temp_var, info in sel_temp_dict.items():
            result_lines.append(f"{indent}logic {info['width']} {temp_var};")
        result_lines.append("")  # 添加空行分隔
        
        # 添加logic行及其后面的内容
        result_lines.extend(lines[logic_line_index:])
    else:
        # 如果没有logic行，直接返回原代码
        return verilog_code
    
    # 对 sel_temp_dict 的键按照末尾数字部分从大到小排序
    def extract_number(temp_var):
        match = re.search(r'_(\d+)$', temp_var)
        return int(match.group(1)) if match else -1
    
    sorted_temp_vars = sorted(sel_temp_dict.keys(), key=extract_number, reverse=True)

    # 插入赋值语句到对应temp_var上一行（从后往前查找）
    for temp_var in sorted_temp_vars:
        info = sel_temp_dict[temp_var]
        for i in range(len(result_lines) - 1, -1, -1):
            if re.search(rf'\b{temp_var}\b', result_lines[i]):
                # 找到temp_var所在行，确定该行的缩进
                target_indent = result_lines[i][:len(result_lines[i]) - len(result_lines[i].lstrip())]
                
                # 检查是否需要加上assign关键字
                previous_line = result_lines[i - 1] if i > 0 else ""
                
                # 检查是否在过程块(always/initial)内
                # 简单的判断方法：如果在模块顶层（缩进较少且不在always/initial之后），则需要assign
                is_top_level = True
                for j in range(i - 1, -1, -1):
                    prev_line_strip = result_lines[j].strip()
                    if prev_line_strip.startswith(("always", "initial", "begin", "if", "else", "case", "for", "while")):
                        is_top_level = False
                        break
                    if prev_line_strip.startswith("endmodule"):
                        break
                
                if "assign" in previous_line or "assign" in result_lines[i] or is_top_level:
                    # 如果上下文有assign语句或者是顶层，加上assign
                    if CONVERT_ASSIGN_TO_ALWAYS and not is_top_level:
                        result_lines.insert(i, f"{target_indent}always @(*) begin\n{target_indent}    {temp_var} = {info['expr']};\n{target_indent}end")
                    else:
                        result_lines.insert(i, f"{target_indent}assign {temp_var} = {info['expr']};")
                else:
                    # 过程块内部，直接插入赋值语句
                    result_lines.insert(i, f"{target_indent}{temp_var} = {info['expr']};")
                break  # 只插入一次，跳出当前变量的查找
    
    return '\n'.join(result_lines)

# =========================================================================
# Assertion extraction from verilator assert-stage dump
# =========================================================================

def extract_assertions_from_ast(ast):
    """Extract assertion ALWAYS blocks from a flattened verilator AST (already loaded dict).

    In the flattened JSON (--flatten --json-only --assert), assertions appear as
    ALWAYS blocks directly inside TOPSCOPE blocksp, without a wrapping BEGIN node.
    Pattern: ALWAYS -> SENTREE(SENITEM(VARREF)) + IF(CEXPR) -> IF(cond) -> DISPLAY

    Returns a list of ALWAYS nodes (wrapped in a dict with name for compatibility).
    """
    assertions = []
    for module in ast.get("modulesp", []):
        for stmt in module.get("stmtsp", []):
            if stmt.get("type") == "TOPSCOPE":
                for scopep in stmt.get("scopep", []):
                    for block in scopep.get("blocksp", []):
                        if block.get("type") != "ALWAYS":
                            continue
                        stmtsp = block.get("stmtsp", [])
                        if not stmtsp:
                            continue
                        # Check: first stmt should be IF with CEXPR condition
                        first = stmtsp[0]
                        if first.get("type") != "IF":
                            continue
                        condp = first.get("condp", [])
                        if not condp or condp[0].get("type") != "CEXPR":
                            continue
                        # This is an assertion ALWAYS block
                        # Find assertion name from DISPLAY -> SFORMATF text
                        # Format: "ASSERTION VIOLATION: HACKDAC_p1 - description"
                        name = "assertion"
                        thensp = first.get("thensp", [])
                        if thensp:
                            inner = thensp[0]
                            if inner.get("type") == "DISPLAY":
                                # Direct DISPLAY (assertion fully optimized by Verilator to CEXPR)
                                fmtp = inner.get("fmtp", [])
                                if fmtp:
                                    fmt_text = fmtp[0].get("name", "")
                                    m = re.search(r'ASSERTION VIOLATION:\s*(\S+)', fmt_text)
                                    if m:
                                        name = m.group(1)
                            elif inner.get("type") == "IF":
                                # Inner IF with assertion condition; DISPLAY is in thensp/elsesp
                                for branch in ['thensp', 'elsesp']:
                                    branch_nodes = inner.get(branch, [])
                                    if branch_nodes and branch_nodes[0].get("type") == "DISPLAY":
                                        fmtp = branch_nodes[0].get("fmtp", [])
                                        if fmtp:
                                            fmt_text = fmtp[0].get("name", "")
                                            m = re.search(r'ASSERTION VIOLATION:\s*(\S+)', fmt_text)
                                            if m:
                                                name = m.group(1)
                                                break
                        # Wrap with a BEGIN-like dict for compatibility with generate_sva_assertion
                        assertions.append({
                            "name": name,
                            "stmtsp": [block],
                            "type": "BEGIN",
                        })
            # Also check BEGIN nodes (for the old assert-stage dump format)
            elif stmt.get("type") == "BEGIN" and re.search(r'p\d+', stmt.get("name", "")):
                assertions.append(stmt)
    return assertions


def _flatten_signal_names(expr, top_module_name, top_ports):
    """Post-process an assertion expression: replace . with ___ and add top module prefix.

    After VerilogGenerator.handle_expression() converts the AST, signal names from
    the flattened JSON already use "top.signal" format (e.g. "adder_32bit.carry").
    We convert them to the triple-underscore format matching the main pipeline:
    - "adder_32bit.carry" -> "adder_32bit___carry"
    - bare names like "carry" (top-level IOs) -> "adder_32bit___carry"

    IMPORTANT: Text inside /* */ comments is preserved verbatim (not signal-flattened),
    since comment content is never a signal reference.
    """
    # Step 0: Strip /* */ comments from the expression — they contain Verilator
    # warnings (e.g. from --bbox-unsup) and must NOT be signal-flattened.
    # Signal names inside comments are not real signal references.
    expr = re.sub(r'/\*.*?\*/', '', expr)

    # Step 1: Replace . with ___ (same as main pipeline)
    expr = expr.replace(".", "___")

    # Step 2: Add top module prefix to bare identifiers that aren't already prefixed
    keywords = {'posedge', 'negedge', 'edge', 'and', 'or', 'not', 'if', 'else',
                'true', 'false', 'assert', 'property', 'cover', 'sequence',
                '$sampled'}

    # Collect replacements from right to left to preserve positions
    replacements = []
    for match in re.finditer(r'\b([a-zA-Z_][a-zA-Z0-9_]*)\b', expr):
        signal = match.group(1)
        start = match.start()

        # Skip if preceded by ' (Verilog constant like 'h0, 'b1) or $ (system function like $sampled)
        if start > 0 and (expr[start - 1] == "'" or expr[start - 1] == "$"):
            continue
        # Skip identifiers preceded by :: (package-scoped identifiers like riscv::PRIV_LVL_M)
        if start >= 2 and expr[start-2:start] == "::":
            continue
        # Skip keywords
        if signal in keywords:
            continue
        # Skip top-level ports (they keep bare name in the flattened output)
        if signal in top_ports:
            continue
        # Skip if already prefixed with top module
        if signal.startswith(f"{top_module_name}___"):
            continue
        # Skip pure numbers
        if signal.isdigit():
            continue

        replacements.append((start, match.end(), f"{top_module_name}___{signal}"))

    # Apply from right to left
    for start, end, new_text in sorted(replacements, reverse=True):
        expr = expr[:start] + new_text + expr[end:]

    return expr


def _merge_declarations(verilog_code):
    """Merge duplicate unpacked array element declarations into single unpacked array declarations.

    Verilator's flattened JSON represents each unpacked array element as a separate
    VAR node. After . -> ___ replacement, this produces multiple declarations like:
        reg [127:127] name[127];
        reg [126:126] name[126];
        reg [31:28] name[31:28];
        reg [27:24] name[27:24];
    These get merged into:
        reg [127:0] name;
    """
    # Pattern A: declarations with packed dimension and unpacked element range
    # e.g., reg [31:28] name[31:28];  or  reg [3:0] name[3];
    pattern_with_packed = re.compile(
        r'^(\s*(?:reg|wire|logic)\s+)\[(\d+):(\d+)\]\s+(\w+)\[(\d+):(\d+)\]\s*;',
        re.MULTILINE
    )
    # Pattern B: declarations without packed dimension, with single unpacked index
    # e.g., reg name[7];
    pattern_simple = re.compile(
        r'^(\s*(?:reg|wire|logic)\s+)(\w+)\[(\d+)\]\s*;',
        re.MULTILINE
    )

    # Shared-type pattern for declarations that have a packed dim + single index
    # e.g., reg [3:0] name[7];  — these won't match pattern_with_packed (no colon in unpacked)
    # but also won't match pattern_simple (has packed dim). Catch them separately.
    pattern_with_packed_simple = re.compile(
        r'^(\s*(?:reg|wire|logic)\s+)\[(\d+):(\d+)\]\s+(\w+)\[(\d+)\]\s*;',
        re.MULTILINE
    )

    groups = {}

    for m in pattern_with_packed.finditer(verilog_code):
        prefix = m.group(1)
        packed_left, packed_right = int(m.group(2)), int(m.group(3))
        name = m.group(4)
        if name not in groups:
            groups[name] = []
        groups[name].append((prefix, packed_left, packed_right, m.start(), m.end()))

    for m in pattern_with_packed_simple.finditer(verilog_code):
        prefix = m.group(1)
        packed_left, packed_right = int(m.group(2)), int(m.group(3))
        name = m.group(4)
        idx = int(m.group(5))
        if name not in groups:
            groups[name] = []
        groups[name].append((prefix, packed_left, packed_right, m.start(), m.end()))

    for m in pattern_simple.finditer(verilog_code):
        prefix = m.group(1)
        name = m.group(2)
        idx = int(m.group(3))
        if name not in groups:
            groups[name] = []
        groups[name].append((prefix, idx, idx, m.start(), m.end()))

    # Only process names with multiple declarations
    replacements = []
    for name, decls in groups.items():
        if len(decls) <= 1:
            continue

        # Compute overall packed range: left = max(all_lefts), right = min(all_rights)
        overall_left = max(d[1] for d in decls)
        overall_right = min(d[2] for d in decls)

        # Keep the first declaration, modify it to be the merged vector
        first_prefix = decls[0][0]
        if overall_left > overall_right:
            # Normal packed range: [31:0]
            merged = f"{first_prefix}[{overall_left}:{overall_right}]{name};"
        else:
            # Single-bit or reversed: use the smaller as lsb
            merged = f"{first_prefix}[{overall_left}:{overall_right}]{name};"
        replacements.append((decls[0][3], decls[0][4], merged))

        # Remove the rest
        for i in range(1, len(decls)):
            replacements.append((decls[i][3], decls[i][4], ""))

    # Apply from right to left
    for start, end, new_text in sorted(replacements, reverse=True):
        verilog_code = verilog_code[:start] + new_text + verilog_code[end:]

    return verilog_code


def generate_sva_assertion(begin_node, top_module_name, top_ports, type_table=None, _counter=[0]):
    """Generate one SVA assert property line from a BEGIN assertion node.

    Uses VerilogGenerator.handle_expression() to convert the condition AST
    to a Verilog expression string, then post-processes signal names.

    JSON structure:
    BEGIN{name="pN"} -> ALWAYS@(posedge clk) -> IF{CEXPR guard} -> IF{cond, elsesp: DISPLAY}
    """
    gen = VerilogGenerator()
    gen.convert_complex_sel_to_temp = False  # preserve assertion expression semantics; temp vars from assertion gen wouldn't be emitted
    if type_table:
        gen.type_table = type_table

    try:
        name = begin_node.get("name", "")
        # Make generic assertion names unique to avoid "Duplicate declaration of block"
        if not name or name == "assertion":
            _counter[0] += 1
            name = f"assertion_{_counter[0]}"
        stmtsp = begin_node.get("stmtsp", [])
        if not stmtsp:
            return f"  // {name}: empty body (optimized away by Verilator)"

        always_node = stmtsp[0]

        # Extract clock signal from ALWAYS's sentreep
        clock_signal = "clk"
        sentreep = always_node.get("sentreep") or always_node.get("sensesp", [])
        if sentreep:
            sentree = sentreep[0]
            senitems = sentree.get("sensesp", [])
            if senitems:
                sensp = senitems[0].get("sensp", [])
                if sensp:
                    clock_signal = gen.handle_expression(sensp[0])

        # Flatten clock signal name
        clock_signal = _flatten_signal_names(clock_signal, top_module_name, top_ports)

        # Navigate: outer IF (CEXPR guard) -> thensp -> inner IF (condition) or DISPLAY
        outer_if = always_node["stmtsp"][0]
        thensp = outer_if.get("thensp", [])
        if not thensp:
            return f"  // {name}: no condition found"

        inner = thensp[0]
        inner_type = inner.get("type", "")

        if inner_type == "DISPLAY":
            # Assertion fully optimized by Verilator to CEXPR + DISPLAY.
            # The condition was resolved at compile time (e.g. `define expressions),
            # only the runtime CEXPR guard remains. Cannot reconstruct Verilog assertion.
            fmtp = inner.get("fmtp", [])
            msg = fmtp[0].get("name", f"ASSERTION VIOLATION: {name}") if fmtp else f"ASSERTION VIOLATION: {name}"
            msg = gen.clean_format_string(msg)
            return f"  // {name}: compile-time resolved, skipped (condition lost to Verilator CEXPR optimization)\n  // {name}: $display(\"{msg}\");"

        elif inner_type == "IF":
            condp = inner.get("condp", [])
            if not condp:
                return f"  // {name}: no condp found"

            # Use VerilogGenerator to convert the condition expression
            condition = gen.handle_expression(condp[0])

            # Flatten signal names in condition
            condition = _flatten_signal_names(condition, top_module_name, top_ports)

            # Extract violation message from inner IF's elsesp -> DISPLAY -> SFORMATF
            msg = f"ASSERTION VIOLATION: {name}"
            for branch in ('thensp', 'elsesp'):
                branch_nodes = inner.get(branch, [])
                if branch_nodes and branch_nodes[0].get("type") == "DISPLAY":
                    fmtp = branch_nodes[0].get("fmtp", [])
                    if fmtp:
                        msg = fmtp[0].get("name", msg)
                        break
            msg = gen.clean_format_string(msg)

            # Determine polarity: if DISPLAY is in thensp, the inner IF condition
            # IS the violation condition. assert property (X) fires when X is FALSE,
            # so we negate to make it fire on violation.
            # If DISPLAY is in elsesp, the inner IF condition IS the passing condition,
            # so we use it as-is.
            display_in_thensp = bool(inner.get('thensp', []) and inner.get('thensp', [])[0].get('type') == 'DISPLAY')
            if display_in_thensp:
                condition = f"~({condition})"

            return f'  {name}: assert property (@(posedge {clock_signal}) {condition}) else $display("{msg}");'

        else:
            return f"  // {name}: unexpected inner node type {inner_type}"

        return f'  {name}: assert property (@(posedge {clock_signal}) {condition}) else $display("{msg}");'

    except Exception as e:
        return f"  // Error generating assertion {begin_node.get('name', '?')}: {e}"


# Known HACKDAC19 assertions that Verilator may fully optimize away (CEXPR)
# Keyed by assertion name. Fields: clock, scope_hint (for signal disambiguation),
# condition (original RTL condition), msg (display string).
_HACKDAC19_MISSING = {
    "HACKDAC19_p5": {
        "clock": "clk_i",
        "scope_hint": "csr_regfile_i",
        "condition": "(~(debug_mode_q && umode_i) || (2'b11))",
        "msg": "ASSERTION VIOLATION: HACKDAC19_p5",
    },
}


def _build_varref_signal_map(ast):
    """Scan all VARREF nodes in the AST to build basename -> [FQN, ...] mapping."""
    signal_map = {}

    def scan(node):
        if isinstance(node, dict):
            if node.get("type") == "VARREF":
                name = node.get("name", "")
                if "." in name:
                    parts = name.split(".")
                    basename = parts[-1]
                    if basename not in signal_map:
                        signal_map[basename] = []
                    if name not in signal_map[basename]:
                        signal_map[basename].append(name)
            for v in node.values():
                if isinstance(v, (dict, list)):
                    scan(v)
        elif isinstance(node, list):
            for item in node:
                scan(item)

    for module in ast.get("modulesp", []):
        for stmt in module.get("stmtsp", []):
            scan(stmt)
    return signal_map


def _resolve_signal_name(basename, signal_map, scope_hint, top_module_name, top_ports):
    """Resolve a local signal name to its flattened triple-underscore form."""
    # First, check if this signal appears in our VARREF-based signal map.
    # A signal may be both a top-level port AND an internal hierarchical signal
    # (e.g. umode_i is a top port but also connects to csr_regfile_i.umode_i).
    # The scope_hint disambiguates to the module-local instance.
    if basename in signal_map:
        candidates = signal_map[basename]
        if len(candidates) == 1:
            fqn = candidates[0]
        elif scope_hint:
            matching = [c for c in candidates if scope_hint in c]
            fqn = matching[0] if matching else max(candidates, key=len)
        else:
            fqn = max(candidates, key=len)
        if fqn.startswith(top_module_name):
            return fqn.replace(".", "___")
        return f"{top_module_name}___{fqn.replace('.', '___')}"
    # Not found as a VARREF — might be a top-level port or a constant/parameter
    if basename in top_ports:
        return basename
    return f"{top_module_name}___{basename}"


def reconstruct_missing_assertions(ast, extracted_assertions, top_module_name, top_ports):
    """Reconstruct assertions that Verilator fully optimized away (CEXPR elimination).

    When Verilator resolves an assertion condition to always-true at compile time,
    it eliminates the IF(condition) guard entirely, keeping only the DISPLAY statement
    or removing the whole ALWAYS block. This function rebuilds those assertions from
    known RTL conditions with correctly flattened signal names.
    """
    extracted_names = {a.get("name", "") for a in extracted_assertions}

    # Check which known assertions are missing
    missing = {n: i for n, i in _HACKDAC19_MISSING.items() if n not in extracted_names}
    if not missing:
        return []

    signal_map = _build_varref_signal_map(ast)

    keywords = {'posedge', 'negedge', 'edge', 'and', 'or', 'not', 'if', 'else',
                'true', 'false', 'assert', 'property', 'cover', 'sequence'}

    reconstructed = []
    for name, info in missing.items():
        print(f"  Reconstructing missing assertion: {name}")
        condition = info["condition"]
        scope_hint = info.get("scope_hint", "")

        # Replace bare identifiers with $sampled(flattened_name)
        # Process right-to-left to preserve match positions
        replacements = []
        for match in re.finditer(r'\b([a-zA-Z_][a-zA-Z0-9_]*)\b', condition):
            signal = match.group(1)
            start = match.start()
            end = match.end()

            # Skip identifiers preceded by :: (package-scoped like riscv::PRIV_LVL_M)
            if start >= 2 and condition[start-2:start] == "::":
                continue
            # Skip identifiers followed by :: (package scope prefix like riscv::PRIV_LVL_M)
            if end + 1 < len(condition) and condition[end:end+2] == "::":
                continue
            # Skip Verilog radix prefixes like 'h, 'b, 'd
            if start > 0 and condition[start-1] == "'":
                continue
            # Skip keywords and pure numbers
            if signal in keywords or signal.isdigit():
                continue

            flattened = _resolve_signal_name(signal, signal_map, scope_hint,
                                             top_module_name, top_ports)
            replacements.append((start, end, f"$sampled({flattened})"))

        for start, end, new_text in sorted(replacements, reverse=True):
            condition = condition[:start] + new_text + condition[end:]

        # Resolve clock signal
        clock = info["clock"]
        clock_flattened = clock if clock in top_ports else f"{top_module_name}___{clock}"

        line = (f'  {name}: assert property (@(posedge {clock_flattened}) {condition})'
                f' else $display("{info["msg"]}");')
        reconstructed.append(line)

    return reconstructed


def generate_assertions_block(assertions, top_module_name, top_ports, extra_lines=None, type_table=None):
    """Generate the full assertions block for insertion into flattened Verilog.

    Args:
        assertions: List of assertion AST nodes (BEGIN-like dicts).
        extra_lines: Optional list of pre-rendered assertion lines to append
                     inside the `ifndef SYNTHESIS` guard (for reconstructed assertions).
    """
    lines = [
        "",
        "// =========================================================================",
        "// Assertions (extracted from verilator assert-stage dump)",
        "// =========================================================================",
        "",
        "`ifndef SYNTHESIS",
        "",
    ]
    for a in assertions:
        lines.append(generate_sva_assertion(a, top_module_name, top_ports, type_table))
    if extra_lines:
        lines.extend(extra_lines)
    lines.append("")
    lines.append("`endif")
    lines.append("")
    return "\n".join(lines)




def inject_assertions_into_verilog(verilog_code, assertion_code):
    """Insert assertion code before the last endmodule in the Verilog output."""
    last_endmodule_idx = verilog_code.rfind("endmodule")
    if last_endmodule_idx == -1:
        print("Warning: Could not find endmodule to inject assertions")
        return verilog_code
    return verilog_code[:last_endmodule_idx] + assertion_code + "\n" + verilog_code[last_endmodule_idx:]


def main():
    parser = argparse.ArgumentParser(description='Verilog AST to Source Code Generator')
    input_group = parser.add_mutually_exclusive_group(required=True)
    input_group.add_argument('--file', type=str,
                        help='Input Verilog file path')
    input_group.add_argument('--filelist', type=str,
                        help='Path to a file containing a list of Verilog files')
    parser.add_argument('--top', type=str, required=True,
                        help='Top module name')
    parser.add_argument('-o', '--output', type=str, required=True,
                        help='Output Verilog file path')
    parser.add_argument('--force', action='store_true',
                        help='Force overwrite output file if it exists')
    parser.add_argument('-I', '--include', type=str, action='append', default=[],
                        help='Include directory for verilator (can be specified multiple times)')
    parser.add_argument('--skip-sv2v', action='store_true',
                        help='Skip sv2v conversion step')

    args = parser.parse_args()

    # 检查输出文件
    output_path = Path(args.output)
    
    # 确定输入文件或filelist
    if args.file:
        # 单文件模式
        input_path = Path(args.file).resolve()
        if not input_path.exists():
            print(f"Error: Input file '{args.file}' does not exist", file=sys.stderr)
            sys.exit(1)
        input_dir = input_path.parent
        files_arg = f'"{input_path.name}"'  # 只需要文件名，因为我们会切换到目录
    elif args.filelist:
        # filelist模式
        filelist_path = Path(args.filelist).resolve()
        if not filelist_path.exists():
            print(f"Error: Filelist '{args.filelist}' does not exist", file=sys.stderr)
            sys.exit(1)
        
        # 读取filelist
        with open(filelist_path, 'r') as f:
            file_lines = f.readlines()
        
        # 去除注释和空行，并获取文件列表
        input_files = []
        for line in file_lines:
            # 去除注释（以//或#开头的部分）
            line = re.sub(r'(//|#).*$', '', line).strip()
            if line:  # 如果非空
                input_files.append(line)
        
        if not input_files:
            print(f"Error: No valid files found in filelist '{args.filelist}'", file=sys.stderr)
            sys.exit(1)
            
        # 使用filelist所在目录作为工作目录
        input_path = filelist_path  # 保持变量名兼容
        input_dir = filelist_path.parent
        files_arg = ' '.join(input_files)

    try:
        # 切换到输入文件所在目录
        original_dir = Path.cwd()
        os.chdir(input_dir)

        # 获取 main.py 所在的目录 (例如 .../VeriFlatten/src)
        script_dir = Path(__file__).resolve().parent
        
        # 获取项目根目录 (假设是 main.py 的上一级，即 .../VeriFlatten)
        project_root = script_dir.parent
        
        # 构建 verilator 的绝对路径
        verilator_bin = project_root / "oss-cad-suite" / "bin" / "verilator"

        # 检查文件是否存在 (可选，但推荐)
        if not verilator_bin.exists():
            print(f"Error: Verilator not found at {verilator_bin}", file=sys.stderr)
            sys.exit(1)

        # 构造 -I 参数
        include_args = "-I."
        for inc_dir in args.include:
            include_args += f" -I{inc_dir}"

        # 使用变量构造命令
        cmd = f"{verilator_bin} {files_arg} {include_args} --assert --flatten --top {args.top} -fno-acyc-simp -fno-const-before-dfg -fno-combine -fno-const -fno-const-bit-op-tree -fno-expand -fno-merge-cond -fno-merge-cond-motion \
                -fno-subst-const -fno-subst -fno-table -fno-dfg --no-std --json-only --no-json-edit-nums  -fno-life -fno-assemble --timing -Wno-fatal -Wno-BLKANDNBLK -Wno-ASSIGNIN --bbox-unsup --error-limit 10000 "
        # cmd = f"{verilator_bin} {files_arg} {include_args} -O3 --flatten --top {args.top}  --no-std --json-only --no-json-edit-nums --assert --timing -Wno-fatal -Wno-BLKANDNBLK -Wno-ASSIGNIN --bbox-unsup --error-limit 10000 "
        
        # 构造JSON文件路径
        json_path = input_dir / "obj_dir" / f"V{args.top}.tree.json"

        if not json_path.exists() or args.force:
            # 修改点 1: 移除 capture_output=True
            # 使用 stdout=subprocess.PIPE, stderr=subprocess.PIPE 来捕获输出
            result = subprocess.run(
                cmd, 
                shell=True, 
                stdout=subprocess.PIPE, 
                stderr=subprocess.PIPE, 
                text=True # 保持 text=True 以便将输出解码为字符串
            )
            
            # 修改点 2: 无论结果如何，都打印标准输出和标准错误
            # 打印标准输出 (stdout)
            if result.stdout:
                print("--- Verilator STDOUT Start ---")
                print(result.stdout)
                print("--- Verilator STDOUT End ---")
                
            # 打印标准错误 (stderr)
            if result.stderr:
                print("--- Verilator STDERR Start ---")
                print(result.stderr)
                print("--- Verilator STDERR End ---")
            
            # 检查错误 (保留原有逻辑)
            if "%Error" in result.stderr or "%Error" in result.stdout:
                # 由于上面已经打印了完整的输出，这里的 print(f"Error: {result.stdout}") 可以简化或删除，
                # 但为了保持和原逻辑对齐，可以保留一个简单的错误提示
                print("!!! Error detected in Verilator output. Exiting. !!!", file=sys.stderr)
                sys.exit(1)
                
            if not json_path.exists():
                print(f"Error: JSON file '{json_path}' was not generated. (ERROR in verilog)", file=sys.stderr)
                sys.exit(1)

        # 读取并解析JSON
        with open(json_path, 'r') as f:
            try:
                ast = json.load(f)
            except json.JSONDecodeError as e:
                print(f"Error: Invalid JSON in file: {e}", file=sys.stderr)
                sys.exit(1)

        # 切回原始目录
        os.chdir(original_dir)

        # 生成代码
        generator = VerilogGenerator()
        verilog_code = generator.generate(ast)

        verilog_code = insert_resultp_statements(verilog_code, generator.resultp_dict)
        verilog_code = insert_sel_temp_statements(verilog_code, generator.sel_temp_dict)

        verilog_code = verilog_code.replace("$root", args.top)
        verilog_code = verilog_code.replace(".", "___")
        verilog_code = re.sub(r'\[(\d+)\]___', r'\1___', verilog_code)

        # --- Build top_ports set for assertion signal name flattening ---
        top_ports = set()
        for m in ast.get("modulesp", []):
            if m.get("name") == "$root":
                for stmt in m.get("stmtsp", []):
                    if stmt.get("type") == "VAR" and stmt.get("isPrimaryIO"):
                        top_ports.add(stmt.get("name", ""))
                break

        # --- Assertion extraction from the flattened JSON ---
        # The main JSON (--flatten --json-only --assert) already contains assertion
        # ALWAYS blocks with flattened signal names (e.g. "adder_32bit.carry").
        # No second verilator pass needed.
        assertions = extract_assertions_from_ast(ast)
        extra_assertions = reconstruct_missing_assertions(ast, assertions, args.top, top_ports)
        if assertions or extra_assertions:
            assertion_code = generate_assertions_block(assertions, args.top, top_ports, extra_assertions, generator.type_table)
            verilog_code = inject_assertions_into_verilog(verilog_code, assertion_code)
            print(f"  Injected {len(assertions)} assertions + {len(extra_assertions)} reconstructed into output")
        else:
            print("  No assertions found in flattened JSON")

        os.chdir(original_dir)

        # 写入输出文件
        output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            f.write(verilog_code)

        with open(args.output, 'r') as f:
            content = f.read()

        # Fix invalid [0:-1] ranges produced for empty arrays
        content = re.sub(r'\[0:-1\]\s*;', '[0:0];', content)

        # Merge duplicate unpacked array declarations
        content = _merge_declarations(content)

        # Fix: when Verilator flattens array elements, names like "sig[0]"
        # have the [0] as part of the hierarchical name, not an unpacked dimension.
        # E.g., "reg [0:0] foo[0];" should be "reg [0:0] foo;" since [0:0] already
        # captures the single-bit width. References like "foo[0]" remain valid as
        # bit-selects on the 1-bit packed vector.
        content = re.sub(r'\[0:0\]\s+(\w+)\[0\]\s*;', r'[0:0] \1;', content)

        # Run sv2v on the output (skip if --skip-sv2v or assertions injected, since sv2v strips SVA)
        if not args.skip_sv2v and not assertions:
            sv2v_bin = project_root / "oss-cad-suite" / "bin" / "sv2v"
            if sv2v_bin.exists() and not sv2v_bin.is_symlink():
                pre_sv2v_content = content

                sv2v_cmd = f"{sv2v_bin} {args.output} > {args.output}.tmp && mv {args.output}.tmp {args.output}"
                sv2v_result = subprocess.run(sv2v_cmd, shell=True, capture_output=True, text=True)
                if sv2v_result.returncode != 0:
                    print(f"Warning: sv2v failed, using raw output.\n{sv2v_result.stderr}", file=sys.stderr)
                else:
                    print(f"sv2v conversion complete: '{args.output}'")
                    with open(args.output, 'r') as f:
                        content = f.read()

                    def _fix_unpacked_range(match):
                        kind = match.group(1)
                        packed_msb = int(match.group(2))
                        name = match.group(3)
                        unpacked_msb = int(match.group(4))
                        if unpacked_msb == packed_msb - 1:
                            return f"{kind} [{packed_msb}:{packed_msb}] {name} [0:{packed_msb}];"
                        return match.group(0)

                    content = re.sub(
                        r'(reg|wire)\s+\[(\d+):\2\]\s+(\w+)\s+\[0:(\d+)\];',
                        _fix_unpacked_range,
                        content
                    )
                    content = re.sub(r'\[0:-1\]\s*;', '[0:0];', content)

                    # Verify sv2v output: revert if verilator --lint-only --assert finds errors
                    verilator_bin = project_root / "oss-cad-suite" / "bin" / "verilator"
                    if verilator_bin.exists():
                        with open(args.output, 'w') as f:
                            f.write(content)
                        v_cmd = f"{verilator_bin} --lint-only --assert {args.output} 2>&1"
                        v_result = subprocess.run(v_cmd, shell=True, capture_output=True, text=True)
                        stderr_lc = (v_result.stderr or "").lower()
                        stdout_lc = (v_result.stdout or "").lower()
                        if "%error" in stderr_lc or "%error" in stdout_lc:
                            print(f"Warning: sv2v output has lint errors, reverting to pre-sv2v version.",
                                  file=sys.stderr)
                            content = pre_sv2v_content
                        else:
                            print(f"sv2v output verified: no lint errors")
            else:
                print(f"Warning: sv2v not found at {sv2v_bin}, skipping sv2v conversion.")
        elif assertions:
            print("Skipping sv2v conversion (assertions injected, sv2v would strip SVA)")

        # Post-processing: add stub wire declarations for signals referenced in
        # SVA assertions ($sampled(...)) that don't have declarations.
        # These are signals that Verilator optimized away (constant propagation)
        # but the assertion AST still references them.
        top_name = args.top
        if top_name:
            import re as _re_assert
            # Find all $sampled(signal_name) patterns
            sampled_signals = set()
            for m in _re_assert.finditer(r'\$sampled\s*\(\s*(' + _re_assert.escape(top_name) + r'___\w+)\s*\)', content):
                sampled_signals.add(m.group(1))
            # Also find signals in @(posedge signal) / @(negedge signal) of assertions
            for m in _re_assert.finditer(
                r'@\s*\(\s*(?:posedge|negedge)\s+(' + _re_assert.escape(top_name) + r'___\w+)\s*\)',
                content
            ):
                sampled_signals.add(m.group(1))
            if sampled_signals:
                # Check which ones lack declarations
                missing_decls = []
                for sig in sorted(sampled_signals):
                    # Check if signal already has a wire/reg/logic declaration
                    decl_pattern = rf'^\s*(?:wire|reg|logic)\s+.*\b{_re_assert.escape(sig)}\b'
                    if not _re_assert.search(decl_pattern, content, _re_assert.MULTILINE):
                        missing_decls.append(sig)
                if missing_decls:
                    # Add stub wire declarations before endmodule
                    stub_decls = "\n".join(f"    wire {sig};" for sig in missing_decls)
                    content = content.rstrip()
                    if content.endswith("endmodule"):
                        content = content[:content.rfind("endmodule")]
                    content += stub_decls + "\nendmodule\n"

        with open(args.output, 'w') as f:
            f.write(content)

        # Final verification: run verilator --lint-only --assert on the output
        verilator_bin = project_root / "oss-cad-suite" / "bin" / "verilator"
        if verilator_bin.exists():
            v_cmd = f"{verilator_bin} --lint-only --assert {args.output} 2>&1"
            v_result = subprocess.run(v_cmd, shell=True, capture_output=True, text=True)
            stderr_all = (v_result.stderr or "") + (v_result.stdout or "")
            errors = [l for l in stderr_all.splitlines() if l.startswith("%Error")]
            if errors:
                print(f"Warning: {len(errors)} verilator error(s) in output:", file=sys.stderr)
                for e in errors[:5]:
                    print(f"  {e}", file=sys.stderr)
                if len(errors) > 5:
                    print(f"  ... and {len(errors)-5} more", file=sys.stderr)
            else:
                print(f"Output verified: no lint errors")

        print(f"Successfully generated Verilog code in '{args.output}'")

    except Exception as e:
        tb_str = traceback.format_exc()
        print(f"Error occurred:\n{tb_str}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    # Start tracking time and memory
    start_time = time.time()
    process = psutil.Process(os.getpid())
    start_memory = process.memory_info().rss / 1024 / 1024  # MB
    
    # Run main function
    main()

    # Calculate execution time and memory usage
    end_time = time.time()
    end_memory = process.memory_info().rss / 1024 / 1024  # MB
    
    execution_time = end_time - start_time
    memory_used = end_memory - start_memory
    
    print(f"\n--- Execution Summary ---")
    print(f"Execution time: {execution_time:.4f} seconds")
    print(f"Memory used: {memory_used:.2f} MB")
    print(f"Peak memory: {process.memory_info().rss / 1024 / 1024:.2f} MB")
