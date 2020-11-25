# consai2r2_examples

consai2r2のサンプルコード集です。

## How To Use Examples

シミュレータでサンプルを実行する場合は[grSim](https://github.com/RoboCup-SSL/grSim)を起動してください。

- [joystick_example](#joystick_example)

---

## joystick_example


ジョイスティックコントローラでgrSim上のロボットを操縦します。
下記のコントローラに対応しています。

- [Logicool Wireless Gamepad F710](https://gaming.logicool.co.jp/ja-jp/products/gamepads/f710-wireless-gamepad.html#940-0001440)
- [SONY DUALSHOCK 3](https://www.jp.playstation.com/ps3/peripheral/cechzc2j.html)

次のコマンドでノードを起動します。

```sh
# 例：/dev/input/js0に接続された F710コントローラを使用
$ ros2 launch consai2r2_examples joystick_example.launch.py dev:=/dev/input/js0 joyconfig:=f710

# 例：/dev/input/js1に接続された DUALSHOCK 3コントローラを使用
$ ros2 launch consai2r2_examples joystick_example.launch.py dev:=/dev/input/js1 joyconfig:=dualshock3
```

### KeyConfig

走行

|Key combination|Function|
|:---|:---|
|`L1` + `L stick ↑↓`|Outputs vel_surge|
|`L1` + `L stick ←→`|Outputs vel_sway|
|`L1` + `R stick ↑↓`|Outputs vel_angular|
|`L1` + `D-Pad ↑↓`|Change max_velocity (0.0 ~ 1.0 * `max_velocity_*`)|
|`L1` + `D-Pad →`|Reset max_velocity (0.5 * `max_velocity_*`)|

---

チームカラーとID

|Key combination|Function|
|:---|:---|
|`L2` + `D-Pad ↑↓`|Change target robot ID (0 ~ `max_id`)|
|`L2` + `D-Pad →`|Toggle target team color (blue or yellow)|

---

キック

|Key combination|Function|
|:---|:---|
|`R1` + `□`|Do straight kick|
|`R1` + `△`|Do chip kick|
|`R1` + `D-Pad ↑↓`|Change kick_power (0.0 ~ 1.0)|
|`R1` + `D-Pad →`|Reset kick_power (0.5)|

---

ドリブル

|Key combination|Function|
|:---|:---|
|`R2` |Drive the dribbler|
|`R2` + `D-Pad ↑↓`|Change dribble_power (0.0 ~ 1.0)|
|`R2` + `D-Pad →`|Reset dribble_power (0.5)|

---

その他

|Key combination|Function|
|:---|:---|
|`SELECT` + `START` |Shutdown the teleop node|

### Configure

キー設定は[consai2r2_teleop/config/joy_f710.yaml](../consai2r2_teleop/config/joy_f710.yaml)、
[consai2r2_teleop/config/joy_dualshock3.yaml](../consai2r2_teleop/config/joy_dualshock3.yaml)で変更できます。

```yaml
consai2r2_teleop:
  ros__parameters:
    button_move_enable      : 4
    axis_vel_surge          : 1
    axis_vel_sway           : 0
    axis_vel_angular        : 3
```

`max_id`や`max_velocity_*`等のパラメータは[consai2r2_description/config/config.yaml](../consai2r2_description/config/config.yaml)で変更できます。

```yaml
consai2r2_description:
  ros__parameters:
    max_id:               15
    our_side:             'left'
    our_color:            'blue'
    max_velocity_surge:   3.0  # m/s
    max_velocity_sway:    3.0  # m/s
    max_velocity_angular: 6.28 # rad/s
```

### Videos

No contents.

[back to example list](#how-to-use-examples)

---
