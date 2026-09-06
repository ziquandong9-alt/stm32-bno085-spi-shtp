# 发布与版本维护检查表

每次推送重要改动、创建 tag 或制作 GitHub Release 前，都应重新执行本检查表。

## 代码与构建

- [ ] `git status` 中只包含计划发布的文件。
- [ ] `git diff --check` 无空白错误。
- [ ] Keil `Rebuild all target files` 为 0 error、0 warning。
- [ ] `.ioc` 中 SPI Mode、分频和 GPIO 与生成的源码一致。
- [ ] 不存在重复的旧驱动目录或同名源文件。
- [ ] 构建产物、用户配置、日志和本地 IDE 文件已被 `.gitignore` 排除。

## 硬件验证

- [ ] 冷上电可稳定读取 Product ID。
- [ ] 连续运行至少 60 秒，Rotation Vector 和 accelerometer 均无 timeout。
- [ ] 静止时三轴加速度模长接近 9.81 m/s²。
- [ ] 转动板子时 yaw/roll/pitch 方向符合项目坐标约定。
- [ ] 拔掉传感器可进入错误处理，重新接入并复位可恢复。
- [ ] 如改变 SPI 频率，使用示波器或逻辑分析仪确认不超过 3 MHz。

## 文档与合规

- [ ] README 中的接线与实际硬件一致。
- [ ] 公共 API、单位、坐标系、包含重力等行为均有说明。
- [ ] LICENSE 中的版权主体和年份由发布者确认。
- [ ] STM32 HAL、CMSIS 等第三方文件保留原版权和许可证。
- [ ] 没有 Wi-Fi 密码、账号、绝对私人路径、芯片密钥或其他敏感信息。
- [ ] 截图、数据手册和其他资源确认允许再分发；优先只放官方链接。

## GitHub 仓库设置

- [ ] 确认仓库名称、Owner、公开/私有属性和简介。
- [ ] 配置 topics，例如 `bno085`、`stm32`、`spi`、`shtp`、`imu`。
- [ ] 创建远端后再次检查 `git remote -v`，确认没有指向错误账号。
- [ ] push 前检查完整 commit diff，而不仅是最后一个文件。
- [ ] 创建 tag/release 时填写版本号、兼容硬件和验证结果。
