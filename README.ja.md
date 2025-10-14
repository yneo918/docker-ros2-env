# docker-ros2-env

ROS 2 開発用のコンテナ環境を手元のワークスペースと共有しながら使うためのテンプレートです。ホスト側のファイルをそのままコンテナ内にマウントし、`docker exec` 経由で `ros2 run` や `ros2 launch` を実行できるようにしています。

## Windows での利用

- 推奨環境は Git Bash です。PowerShell ではシェルスクリプトの互換性に注意してください。
- 事前に `git config --global core.autocrlf false` を実行し、LF 改行を維持してください。既にクローン済みの場合は LF で再取得するか `git rm --cached -r . && git reset --hard` で正規化します。
- 使用前に Docker Desktop を起動し、バックグラウンドで稼働させてから compose やスクリプトを呼び出してください。

## 構成概要

- ベースイメージに `ros:jazzy-ros-base` を使用（`docker compose build --build-arg ROS_DISTRO=humble` などで変更可能）。
- `workspace/` ディレクトリを `/workspaces/host_ws` としてマウントし、ホストとコンテナで同じソース・ビルド成果物を共有。
- `docker compose up -d` で常駐する ROS 2 コンテナを起動し、複数ターミナルから `docker exec` で再利用。
- エントリポイントがホストの UID/GID に一致するユーザーをコンテナ内に作成し、`./scripts/container_shell.sh` や `ros2_exec.sh` 実行時もそのユーザーを利用。
- `scripts/ros2_exec.sh` で `ros2 ...` サブコマンドをラップし、毎回セットアップを記述せずに実行。

## 使い方

1. **ビルド**
   ```bash
   docker compose build ros2
   ```
   - 他のディストリビューションを使いたい場合は `docker compose build --build-arg ROS_DISTRO=humble ros2` のように指定し、あわせて `.env` の `ROS_DISTRO` も更新してください。

2. **コンテナの常駐起動**
   ```bash
   docker compose up -d ros2
   ```

3. **`ros2` コマンドの実行**
   - 例: `ros2 run demo_nodes_cpp talker`
     ```bash
     ./scripts/ros2_exec.sh run demo_nodes_cpp talker
     ```
   - 例: `ros2 launch demo_nodes_cpp talker_listener.launch.py`
     ```bash
     ./scripts/ros2_exec.sh launch demo_nodes_cpp talker_listener.launch.py
     ```

4. **コンテナ内のシェルへ入る**（`colcon build` など汎用コマンド用）
   ```bash
   ./scripts/container_shell.sh
   ```
   - シェルに入るユーザーはホストと同じ UID/GID に自動調整され、マウントしたファイルの所有権が崩れません。

5. **終了**
   ```bash
   docker compose stop ros2
   ```

> `docker exec ros2_dev bash -lc "ros2 run ..."` のように直接コマンドを実行したい場合も、同じコンテナ名と `source /opt/ros/$ROS_DISTRO/setup.bash` の手順で実行できます。

## ワークスペースの準備

- `workspace/` 以下は通常の ROS 2 ワークスペース構成（例: `workspace/src/...`）。
- 初回はコンテナ内で `colcon build` を実行し、生成された `install/` を共有したまま再利用できます。
- 別のワークスペースを使いたい場合は `docker-compose.yml` のボリューム設定を編集してください。
- Python パッケージを追加したい場合は `requirements.txt` に追記し、`docker compose build ros2` で再ビルドするとコンテナ内にインストールされます（PEP 668 の制限は Dockerfile で解除済み）。

## コンテナ設定

- 主要な環境変数やコンテナ名は `.env` で管理しています。
  - `ROS2_UID` / `ROS2_GID`: コンテナ内で作成するユーザーの ID。ホストの `id -u` / `id -g` を設定すると、エントリポイントが一致するユーザーを生成します（未設定時はシェルスクリプトがホストの ID を自動で使用）。
  - `ROS2_USERNAME` / `ROS2_GROUPNAME`（任意）: UID/GID を割り当てる際のユーザー名とグループ名。デフォルトは `ros2`。
  - `ROS_DISTRO`: 使用する ROS ディストリビューション（デフォルト `jazzy`。ビルド時に `--build-arg ROS_DISTRO=...` で変更可能）。
  - `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION`: DDS 関連設定。
  - `ROS2_CONTAINER_NAME`: `docker exec` 等で参照するコンテナ名。
- Display を共有して RViz などを使いたい場合は、ホストで `xhost +local:` を実行するなど X11 の受け入れ設定を行ってください。

## 追加インストール済みの主要 ROS パッケージ

コンテナではベースの `ros-base` に加えて、シミュレーションやテレオペを動かすためのパッケージを追加インストールしています。

- `ros-${ROS_DISTRO}-demo-nodes-cpp`
- `ros-${ROS_DISTRO}-xacro`
- `ros-${ROS_DISTRO}-rviz2`
- `ros-${ROS_DISTRO}-joy`

これらは `colcon build` や launch ファイルからの依存解決で利用されます。新たに必要なパッケージが出た場合は `Dockerfile` に追記し、`docker compose build ros2` を再実行してください。

## よく使うパターン

- 複数のターミナルで同じノード群を操作する際は、最初に `docker compose up -d ros2` でコンテナを起動しておき、各ターミナルで `./scripts/ros2_exec.sh ...` を呼び出します。
- 新しい依存関係を追加したい場合は、`Dockerfile` を編集し再ビルドしてください。
- コンテナを再作成してもホスト側のビルド成果物は `workspace/` に残るため、`docker compose down -v` をしない限り削除されません。
