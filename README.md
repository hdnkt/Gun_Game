# Gun_Game
RaspberryPiとカメラを使ってガンシューティングゲームコントローラーと，それ用の（申し訳程度の）ゲームを作りました．（2018,2019）学祭展示

## できること
### 実世界拡張
- あらかじめ用意しておいた「風船」「空き缶」といったオブジェクトに照準を合わせて引き金を引くと，「風船が割れる」「空き缶が吹っ飛ぶ」といった演出がおきる．
- （「風船が割れると敵が全滅（シューティングゲームのボム扱い）」のようにゲームと絡めることで没入感を上げるという話があったのだが，結局良い案は出ずじまい）
### 多面シューティング 
- 銃型コントローラーをゲーム画面に向けて引き金を引くと弾丸がそこに出て，シューティングゲームができる．
- ゲーム画面は複数用意でき（２画面），「両方から敵が攻めてくる」ようなゲームを体験できる．

## 使った物
- コントローラー
  - RaspberryPi3
  - RaspberryPi3のカメラモジュール
  - Redbear(通信用マイコン)
- 画面
  - ゲームの進行と画面描画するPC 
  - プロジェクター/1画面
  - 赤外光LEDを5個/1画面
- 「割れる風船」
  - 風船（展示の時はたくさん用意する）
  - ardiuno
  - Redbear(通信用マイコン)
  - 赤外光LEDたくさん
  - 針
  - ソレノイド
- 「飛ぶ空き缶」
  - 空き缶
  - ardiuno
  - Redbear(通信用マイコン)
  - ソレノイド
  - バネ

## 原理
- オブジェクトの判定
  - どのオブジェクト（例えば風船A，風船B，空き缶A，空き缶Bがあるうちの風船A）を狙ったのかというのを判定する必要がある．
  - 赤外光が発光するタイミング制御によって判定していた．時間の流れで説明すると下のような感じ
   1. 引き金を引く
   2. 3と4をオブジェクトの数N回繰り返す
   3. オブジェクトiが発光（赤外光なので人の目には見えない）
   4. カメラがシャッターを切り，画像iを保存
   5. 閾値処理した画像iの真ん中に白く光がある程度大きく映っていたらオブジェクトiにヒットしたと判定する
- 画面の座標の計算
  - ゲーム画面に向けて引き金を引いたとき， ゲーム画面の「どこ」を向けて引き金を引いたのかを画面に取り付けた赤外光LEDの画像への映り方によって事前にキャリブレーションした情報を使って座標変換．複数画面のときは取り付ける配置を異なるようにしておいて簡単な画像処理でどちらなのか判定する．

## システム構成
図を貼り付ける

## 上がってるソース
僕（hdnkt）が書いたのはあたり判定などを行うC++のコードとUnityのC#コードです．あたり判定は別言語別環境で書かれていたものを移植しました．
- atarihantei.cpp:コントローラーとしてつかうRasberryPi3で動かすプログラムです．風船や空き缶への当たり判定とゲーム画面のどの座標を狙っているかの計算を行い，メインのPCにパケットを送信します．今読むとかなりべた書き感が強いです（mainが長いよ）（作ったのは学部2年時）．
- Unityのもろもろ:これは別のレポジトリにあげてあります．

## 上がってないソース
- ardiunoでオブジェクトを光らせたりソレノイドを動かして割ったりするコード
- redbearでの通信部分（他の人が作ったのと，手元にないので）
  - 多面シューティングゲームをするだけならこの部分はいらない 

## デバッグ動画
- 覗き込んだところにちゃんと弾が出ている．
https://www.youtube.com/watch?v=lUyStWEtsS0
- ゲーム画面のデバッグ動画．それぞれの画面を教室の前とうしろに移して遊んだ．
https://www.youtube.com/watch?v=GQsivJgEkBI
## 関連URL
- サークルのHP。僕がプロジェクトを引き継ぐ前の代のきれいな写真が載ってるのでイメージだけ参考に．ここに載ってる学術発表などには関わっていません．
https://imedia-lab.net/iml-projects/gun-project/
