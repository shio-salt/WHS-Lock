# WHS-Lock
### 概要
学校のロッカー専用のスマートロックです。Wは"Waseda"、Hは"Honjo"、Sは"Smart"を意味しています。

### 機能
- 指紋認証による施錠・解錠
- アプリによる施錠・解錠・設定(不安定のため、現在削除中)
- オートロック
- 現在時刻の表示

### スリープ
人感センサーが1分間何も感知しないと、施錠して、スリープする。
スリープに入る前には、施錠する。(オートロック)

### メンテナンス
1日1回自動で実行される。主にインターネットを利用した時刻調整を行う。

### 仕様
- マイコン
  + ESP32

- 言語
  + C++
  + (Arduino)

- 部品
  + マイコン: ESP32
  + 指紋認証センサー: R307
  + OLED ディスプレイ: SSD1331
  + サーボモーター: SG92R
  + 人感センサー: SB412A
  + RTC: RTC-8564NB

- 筐体(ケース)
  + 3Dプリンターで作成。素材はPLA。

- 消費電力
  + 起動中: 180mA
  + 動作中: 120mA
  + スリープ中: 3mA
  + 理論上、単3電池4本で20日程度持つ。(1日2時間動作していると仮定した場合)

- アプリ
　+ https://github.com/shio-salt/S-App-Alpha

### 動画
https://www.youtube.com/watch?v=2w3Ie2t5ZWQ

### 連絡先
  + Twitter: @Shio_Salt_0213
  + Discord: @Shio#8560
