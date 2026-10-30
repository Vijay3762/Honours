# Add the VeeR-EL2 RTL and generated configuration to an OPEN Vivado project.
# Run from Vivado Tcl Console:
#   source [file normalize "/path/to/Honours/script/add_veer_el2_to_vivado.tcl"]
#
# This adds the CPU RTL sources only. It does not create the SoC integration top
# or connect the CPU AXI interfaces to the project interconnect.

set script_dir [file dirname [file normalize [info script]]]
set project_root [file normalize [file join $script_dir ..]]
set cpu_root [file join $project_root rtl cpu veer_el2]
set config_dir [file join $cpu_root config]
set flist_path [file join $cpu_root flist.vcs]

if {![file exists $flist_path]} {
    error "VeeR source list not found: $flist_path"
}
if {![file exists [file join $config_dir common_defines.vh]]} {
    error "Generated VeeR configuration missing in $config_dir"
}
if {[current_project -quiet] eq ""} {
    error "Open or create a Vivado project before sourcing this script."
}

# Add required include directories while preserving existing project settings.
set include_dirs [list \
    $config_dir \
    [file join $cpu_root include] \
    [file join $cpu_root lib] \
    [file join $cpu_root dmi]]
set old_include_dirs [get_property include_dirs [current_fileset]]
set_property include_dirs [concat $old_include_dirs $include_dirs] [current_fileset]

# Parse the VCS-style file list, keeping the source order and resolving paths
# relative to the Honours project root.
set fh [open $flist_path r]
set source_paths {}
while {[gets $fh line] >= 0} {
    set line [string trim $line]
    if {$line eq "" || [string match "#*" $line]} {
        continue
    }
    if {[string match "+incdir+*" $line]} {
        continue
    }
    if {[string match "-v *" $line]} {
        set line [string trim [string range $line 3 end]]
    }
    set source_path [file normalize [file join $project_root $line]]
    if {![file exists $source_path]} {
        close $fh
        error "Listed VeeR source file does not exist: $source_path"
    }
    lappend source_paths $source_path
}
close $fh

foreach source_path $source_paths {
    if {[llength [get_files -quiet $source_path]] == 0} {
        add_files -norecurse $source_path
    }
}

# Make the generated defines available to every RTL compilation unit.
set common_defines [file join $config_dir common_defines.vh]
if {[llength [get_files -quiet $common_defines]] == 0} {
    add_files -norecurse $common_defines
}
set_property is_global_include true [get_files $common_defines]

puts "Added [llength $source_paths] VeeR-EL2 RTL source files."
puts "VeeR configuration include directory: $config_dir"
puts "Note: SoC AXI wiring and a top-level module still need to be added."
