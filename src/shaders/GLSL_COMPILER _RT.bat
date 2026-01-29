C:\VulkanSDK\1.4.321.1\Bin\glslc.exe --target-spv=spv1.4 -fshader-stage=rgen  rt_raygen.rgen  -o rt_raygen.spv
C:\VulkanSDK\1.4.321.1\Bin\glslc.exe --target-spv=spv1.4 -fshader-stage=rmiss rt_miss.rmiss   -o rt_miss.spv
C:\VulkanSDK\1.4.321.1\Bin\glslc.exe --target-spv=spv1.4 -fshader-stage=rchit rt_hit.rhit   -o rt_hit.spv
C:\VulkanSDK\1.4.321.1\Bin\glslc.exe --target-spv=spv1.4 -fshader-stage=rchit rt_shadow_hit.rshit   -o rt_shadow_hit.spv
C:\VulkanSDK\1.4.321.1\Bin\glslc.exe --target-spv=spv1.4 -fshader-stage=rmiss rt_shadow_miss.rsmiss   -o rt_shadow_miss.spv
pause