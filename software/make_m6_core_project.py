#!/usr/bin/env python3
"""Generate the TD project for the full-problem M6 DDR2 core top."""
import os

root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
out = os.path.join(root, "anlogic", "td_project", "syn_m6_core")
os.makedirs(out, exist_ok=True)

files = [
    "rtl/top/gfold_m6_core_top.v",
    "rtl/board/ddr2_sram_ctrl.v",
    "rtl/board/eram_spram.v",
    "rtl/board/sram64_ctrl.v",
    "rtl/flash_ctrl.v",
    "rtl/solver/fp64.v",
    "rtl/solver/fp64_cmp.v",
    "rtl/solver/fp64_log.v",
    "rtl/solver/fp64_rsqrt.v",
    "rtl/solver/fp64_pool.v",
    "rtl/solver/drs_iter.v",
    "rtl/solver/kkt_solve.v",
    "rtl/solver/banded_ldl_fp64_rb.v",
    "rtl/solver/spmv_fp64.v",
    "rtl/solver/root_plus_pool.v",
    "rtl/solver/proj_soc_pool.v",
    "rtl/solver/proj_dual_cone_rb.v",
    "rtl/solver/s_build.v",
    "anlogic/ip/ddr2/ph1p35_324_ddr_wrapper.v",
]

hdr = '''<?xml version="1.0" encoding="UTF-8"?>
<Project Version="3" Minor="2">
    <Project_Created_Time></Project_Created_Time>
    <TD_Encoding>UTF-8</TD_Encoding>
    <TD_Version>6.2.168116</TD_Version>
    <Name>gfold_m6_core</Name>
    <HardWare>
        <Family>PH1</Family>
        <Device>PH1P35MDG324</Device>
        <Speed>-3</Speed>
    </HardWare>
    <Source_Files>
        <Verilog>
'''
body = ""
for i, f in enumerate(files, 1):
    body += f'''            <File Path="../../../{f}">
                <FileInfo>
                    <Attr Name="UsedInSyn" Val="true"/>
                    <Attr Name="UsedInP&R" Val="true"/>
                    <Attr Name="BelongTo" Val="design_1"/>
                    <Attr Name="CompileOrder" Val="{i}"/>
                </FileInfo>
            </File>
'''
tail = '''        </Verilog>
        <ADC_FILE>
            <File Path="../../constraints/pin_ddr2.adc">
                <FileInfo>
                    <Attr Name="UsedInSyn" Val="true"/>
                    <Attr Name="UsedInP&R" Val="true"/>
                    <Attr Name="BelongTo" Val="constraint_1"/>
                    <Attr Name="CompileOrder" Val="1"/>
                </FileInfo>
            </File>
        </ADC_FILE>
        <SDC_FILE>
            <File Path="../../constraints/timing_m6.sdc">
                <FileInfo>
                    <Attr Name="UsedInSyn" Val="true"/>
                    <Attr Name="UsedInP&R" Val="true"/>
                    <Attr Name="BelongTo" Val="constraint_1"/>
                    <Attr Name="CompileOrder" Val="2"/>
                </FileInfo>
            </File>
        </SDC_FILE>
        <IP_FILE>
            <File Path="../../ip/ddr2/ddr2/ddr2.xml">
                <FileInfo>
                    <Attr Name="UsedInSyn" Val="true"/>
                    <Attr Name="UsedInP&R" Val="true"/>
                    <Attr Name="BelongTo" Val="design_1"/>
                    <Attr Name="CompileOrder" Val="13"/>
                </FileInfo>
            </File>
        </IP_FILE>
    </Source_Files>
    <FileSets>
        <FileSet Name="design_1" Type="DesignFiles">
        </FileSet>
        <FileSet Name="constraint_1" Type="ConstrainFiles">
        </FileSet>
    </FileSets>
    <TOP_MODULE>
        <LABEL>gfold_m6_core_top</LABEL>
        <MODULE>gfold_m6_core_top</MODULE>
        <CREATEINDEX>user</CREATEINDEX>
    </TOP_MODULE>
    <Property>
    </Property>
    <Device_Settings>
    </Device_Settings>
    <Configurations>
        <Control0>
            <mclk_freq>33MHz</mclk_freq>
        </Control0>
        <FeatureRow>
            <boot_mode>mspix4</boot_mode>
        </FeatureRow>
    </Configurations>
    <Runs>
        <Run Name="syn_1" Type="Synthesis" ConstraintSet="constraint_1" Description="" Active="true" useDefaultScript="false">
            <Strategy Name="Default_Synthesis_Strategy">
            </Strategy>
            <UserParams>
            </UserParams>
        </Run>
        <Run Name="phy_1" Type="PhysicalDesign" ConstraintSet="constraint_1" Description="" SynRun="syn_1" Active="true">
            <Strategy Name="Default_PhysicalDesign_Strategy">
                <BitgenProperty::GeneralOption>
                    <compress>on</compress>
                </BitgenProperty::GeneralOption>
            </Strategy>
            <UserParams>
            </UserParams>
        </Run>
    </Runs>
    <Project_Settings>
    </Project_Settings>
</Project>
'''
with open(os.path.join(out, "gfold_m6_core.prj"), "w", encoding="utf-8") as f:
    f.write(hdr + body + tail)
with open(os.path.join(out, "settings.cfg"), "w", encoding="utf-8") as f:
    f.write('''# gfold_m6_core / syn_1
set ADCList {"../../constraints/pin_ddr2.adc"}
set IpADCList { ddr2 ../../ip/ddr2/ddr2/src/adc/ddr2.adc }
set IpSDCList { ddr2 ../../ip/ddr2/ddr2/src/sdc/ddr2.sdc }
set SDCList {"../../constraints/timing_m6.sdc"}
set area_option -packarea
set device_name ph1_35p.db
set end_step opt_gate
set package_name PH1P35MDG324
set prj_name {gfold_m6_core}
set run_type syn
set speed 3
set start_step read_design
set top_model_name {gfold_m6_core_top}
''')
print("created", out, "with", len(files), "source files")
