# mocopi_ros
mocopiのUDP通信を直接受信してTFに流します。  
ボーンの構成は以下に準拠しています。  
https://www.sony.net/Products/mocopi-dev/jp/documents/Home/TechSpec.html

## 使い方

mocopi_rosを起動すると、待ち受け状態になります。  
```
rosrun mocopi_ros mocopi_receiver.py
```

mocopiアプリの設定画面から「PC接続設定」を選択し、IPアドレス、送信ポート（デフォルト：12351）を入力します。  
送信フォーマットは「mocopi(UDP)」を選択してください。  

モーションの送信を開始し、TF等で動作を確認してください。

## 動作環境
ROS Noetic

## 謝辞
seagetchさんのmcp_receiver(https://github.com/seagetch/mcp-receiver)を引用しています。  
ありがとうございます。  