# BDCMD01
このプロジェクトは，strvが頒布するモータドライバ制御用のソフトウェアです．

# できること
2つのDCモータを楽しく回せます．IMUが載っているので移動ロボットに使うとより楽しくなれます．

# 開発環境
- TrueStudio
- CubeMX

CubeMXで初期設定をしてTrueStudioのプロジェクトを出力し，TrueStudioで開発しています．

# ハードウェア
そのうち本家サイトにアップします．

# コンソール
各種テスト用にコンソールが使えます．
460800bps 8N1でポートを開き，エンターを押すごとにコマンドが実行できます．
「?」とするとインストールしてあるコマンドのヘルプが列挙されます(が，処理がおかしくて文字がかけます)．
また，空白のままエンターを押すと，前回のコマンドを再度実行します．

プログラムファイルとしては，uart_utilが該当しますが，コンソールで利用できるコマンドについては，main.cに書いてあります．
main.cを見れば使い方はわかるかと思いますが，コマンドを処理する関数と，コンソール処理関数に渡すための配列を用意し，コンソール処理関数にpushします．

## トルク(電流)制御
```
tcen 1 
```
上記のコマンドで，モータ1ch側のトルク制御器が有効になります．その後
```
ccmd 1 50
```
とすることで，目標電流を50mAに指定できます．

## 速度制御
```
scen 1
```
上記のコマンドでモータ1ch側の速度制御が有効になります．その後
```
rpm 1 600
```
とすれば，目標回転数を600rpmに指定できます．

# 事前準備
速度制御やトルク制御をするためには，パラメータ設定が必要です．特にゲイン設定は必須です．
ゲインは次のコマンドでそれぞれ指定が可能です．
```
tcgain
scgain
```

ゲインの決め方については，とりあえずゆっくりPを上げていって，発振したらPを2/3くらいにして，そのあとIを上げていきます．
小型モータであれば，電流制御ゲインは
- P : 1000
- I : 100000
- D : 0
程度にしておくととりあえず動くかと思います．

# 開発方@



