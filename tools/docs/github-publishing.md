# 公开发布流程

[返回 README](../../README.md#publish)

开发工作区可以同时保存公开和私有资源。**只发布经过白名单导出、扫描与复核的独立副本。不要推送开发工作区的 Git 历史。** `.gitignore` 不能从已跟踪文件、旧提交、标签、截图或模型中移除信息。

## 生成干净副本

维护者在本机 `.private/publication.json` 保存发布策略，此文件不进入公开副本。策略显式指定允许的源码、机器人和策略目录，排除归档、私有测试、旧截图与生成文件。新增机器人或文件不应自动获得公开授权。

```bash
# 在本地开发目录执行；只检查，不复制、不联网
python3 tools/export_public.py --check

# 输出目录必须不存在；请使用新名字，避免和旧副本混合
python3 tools/export_public.py --output tools/build/github-public

# 独立复查清单、内容、私有资产指纹与文件完整性
python3 tools/export_public.py --audit tools/build/github-public
```

导出工具只复制允许的文件，不复制 `.git`、任务、视频、构建结果、私有发布规则或本机环境。文本及路径含受保护标识、文件与受保护模型/mesh 指纹相同、未经确认的二进制或图片发生变更时，导出会失败，不生成可发布目录。图片还需人工查看；内容扫描不能证明任意资产都获得了分发授权。

副本中的 `PUBLIC_RELEASE.json` 只列公开文件的路径、体积和 SHA-256，不包含被排除文件的名称清单。公开副本不附带本机私有发布规则；其他维护者应制定自己的白名单，不能沿用不存在的本地授权。

## 验证副本

```bash
cd tools/build/github-public
./build.sh core -DCMAKE_BUILD_TYPE=Release
(cd tools/build/core && ctest --output-on-failure)
tools/build/core/rlsim_policy_run \
  'configs/wheel/go2w.yaml#/policies/go2w-ts-v001' "$PWD" --check
/usr/bin/python3 tools/rlsim.py assets --kind robot
```

安装与 ROS1 构建见 README。检查文档图片、相对链接和运行资源是否齐全；模型接口通过不等于闭环动力学验证。构建输出仍由副本的 `.gitignore` 排除，重新审计原始发布清单时应使用未构建的干净副本。

## 创建新的 Git 历史

人工复核公开副本及资产许可后，**只在副本目录**初始化新仓库。不要复制开发目录的 `.git`，也不要从原仓库添加远端并推送旧分支。

```bash
# 确认当前目录是刚生成的发布副本
# 先执行下面的目录检查，失败时停止后续操作
[ -f PUBLIC_RELEASE.json ] && [ ! -e .git ] || exit 1
git init -b main
git add .
git diff --cached --stat
git diff --cached --check
git commit -m "Initial public release"
git remote add origin https://github.com/YOUR_ACCOUNT/YOUR_REPOSITORY.git
git push -u origin main
```

这些命令由发布者在审核后手动执行，导出脚本不会提交或上传。大模型和网格可按实际文件大小选择 Git LFS 或独立资源包；需保留许可证、来源和可复现的资源获取方式。

如果受保护内容此前已经推送到远端，需要另外检查远端分支、标签、Release 与缓存；本地导出和忽略规则不会撤回已有远端数据。
