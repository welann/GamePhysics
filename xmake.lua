set_project("GamePhysics")

add_rules("mode.release","mode.debug")
   
add_includedirs("code")
add_includedirs("$(projectdir)/libs/glfw-3.2.1.bin.WIN64/include")
add_includedirs("$(projectdir)/libs/vulkan_1.1.108.0/Include")

add_linkdirs("$(projectdir)/libs/vulkan_1.1.108.0/Lib","$(projectdir)/libs/glfw-3.2.1.bin.WIN64/lib-vc2015")

target("vulkanrenderer")
    set_kind("binary")
    set_rundir("$(projectdir)")
    add_files("code/**.cpp")
    add_links("vulkan-1","glfw3dll")
    

    -- add_packages("vulkansdk","glfw")
