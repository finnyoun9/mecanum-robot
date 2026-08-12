# experiments/

实验配置、原始数据、波形与结论。所有结论必须可追溯到数据，不使用"运行正常"代替测量。

- 单舵机：ID 扫描、反馈项、延迟/超时/重试 CSV。
- 总线：控制周期、P99 latency、checksum error、timeout、recovery time。
- RTOS：各任务 stack high-water mark、最小剩余 heap、watchdog 验证。
- 机械：工作空间、关节回差、重复定位误差。
- 系统：连续 10 次抓取成功率与失败分类。
