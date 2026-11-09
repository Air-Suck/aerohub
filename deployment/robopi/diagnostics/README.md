# 板端独立诊断

这些脚本用于缩小板端加载、TF、DDS 或启动故障，不参与常规 `up`。
在部署的板端私有目录加载 `board_env.sh` 后运行，完整流程使用
[RoboPi 统一入口](../README.md)。

`full_smoke.py` 沿用早期探针约定，需要在含 `run_as2.py`、配置及 overlay 的
板端工作目录执行；不能直接在本目录调用并假设环境已经准备好。

### 只读时钟观测

```bash
python3 deployment/robopi/diagnostics/clock_probe.py \
  --samples 12 --interval 5 --output log/airsim/clock-observation-01
```

记录两侧墙钟、单调时钟、HDC 往返时间，以及偏差上下界。不调用对时、不重启服务。
退出码 1 表示明确超出 0.5 秒偏差或检测到超过 50 毫秒的墙钟跳变；退出码 0
仅表示本段观察未触发这两个条件，不能证明长期同步可靠。偏差变化小于通信不确定
范围时，漂移率保持未定；短观测不能排除间歇问题。墙钟与单调时钟之差用于检测
跳变，不覆盖两者共同发生的频率校正。测量工具本身的阈值不是飞控验收阈值。
