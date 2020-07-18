# Contribution Guide

consai2r2へのコントリビュート方法について記載してます。

## Issues

いつもconsai2r2をご利用いただきありがとうございます。

issueの内容については制限を設けていません。
将来的にはissueテンプレートを用意したいです。

## Pull Requests

プルリク大歓迎です。

あなたが作成したPull Requestの内容には[MIT ライセンス](./LICENSE)が
適用されることをご理解ください。

[consai2r2 プロジェクト](https://github.com/SSL-Roots/consai2r2/projects)に
適さないPull Requestは拒否される場合があることをご理解ください。

本リポジトリは[GitHub Actions](https://github.com/SSL-Roots/consai2r2/actions)
でコードをチェックしています。
チェックが通らないPull Requestは拒否される場合があることをご理解ください。

## コーディングスタイル

パッケージの構成やコーディングスタイルについては
[ROS 2 Developer Guide](https://index.ros.org/doc/ros2/Contributing/Developer-Guide/)
に従います。

ファイルのフォーマットは
[ament_lint](https://github.com/ament/ament_lint)
によってビルド時にチェックされます。

ローカルでフォーマットをチェックする場合は次のコマンドを実行します。

```sh
$ cd ~/ros2_ws
$ colcon test
# テストの結果を表示
$ colcon test-result --verbose
```

また、次のようにファイルを個別にチェックすることも可能です。

```sh
# C++のコードはament_uncrustifyやament_clang_formatによってフォーマットをチェックできます
$ cd ~/ros2_ws/src/consai2r2
# チェック結果を表示
$ ament_uncrustify consai2r2_teleop/src/joystick_component.cpp           
# フォーマットを修正（ファイルの中身が書き換わります）
$ ament_uncrustify --reformat consai2r2_teleop/src/joystick_component.cpp
```

`consai2r2`のホームディレクトリには[clang-formatの設定ファイル](./.clang-format)を用意しています。
clang-formatコマンドや、エディタのフォーマッタ設定等に活用してください。

`.clang-format`ファイルを使用しても`colcon test`に失敗する場合があります。
**[GitHub Actions](https://github.com/SSL-Roots/consai2r2/actions)でのテストが通るように修正してください。**


## その他

その他気になることがあれば随時issueを立ててください。

