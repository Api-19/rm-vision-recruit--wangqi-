# 附加题二：ARM与x86、NPU与GPU

ARM和x86首先是指令集体系结构，规定程序可见的指令、寄存器及执行规则；芯片的实际性能还受微架构、工艺、缓存和内存影响。ARM采用RISC设计，x86指令编码和历史兼容负担更复杂；不能据此直接断言任意ARM都比任意x86省电。两者二进制一般不能直接互换，需要相应编译或翻译层。[Arm架构说明](https://www.arm.com/architecture/cpu)、[Intel架构手册](https://www.intel.com/content/www/us/en/developer/articles/technical/intel-sdm.html)。

机器人选择ARM平台时，一个常见工程理由是可在有限供电、散热和空间内集成CPU与外设；选型仍应测量相机/雷达接口、驱动兼容、端到端延迟和功耗，而非只看指令集名称。这是工程选型分析，并非“所有机器人必须ARM”的结论。

| 比较 | GPU | NPU |
|---|---|---|
| 主要取向 | 通用并行计算，也可训练和推理 | 神经网络常用算子及数据流的专用加速 |
| 灵活性 | 适合多种并行任务 | 更依赖支持的算子、精度与编译工具 |
| 部署关注点 | 内存传输、功耗、批量与延迟 | 量化精度、算子回退、模型转换、片上存储 |

异构平台通常让CPU负责流程与控制，GPU或NPU承担适合的计算；专用加速能否兑现效率必须在目标模型上测试。[Arm对CPU/GPU/NPU工作负载的说明](https://developer.arm.com/community/arm-community-blogs/b/architectures-and-processors-blog/posts/what-is-the-best-ip-for-machine-learning-workloads)。RoboMaster中应同时计入采集、预处理、推理、后处理和通信时间；TOPS不能替代整条链路的延迟证据。

来源核验：2026-10-19。AI辅助撰写，未进行本机NPU/GPU速度或功耗对比。
