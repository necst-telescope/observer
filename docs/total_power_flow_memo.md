# total_power 描画フローメモ

## 全体の流れ

1. qlook画面の `total_power` ボタン押下
2. フロントエンドが `role="total_power"` でトピック一覧要求
3. サーバーが `quick_spectra` を含むトピックに絞って返却
4. ユーザーがトピックとフィールドを選択
5. フィールド選択でROSトピック購読開始
6. ROSメッセージをWebSocketで `ros2-message` として受信
7. 配列フィールドの総和を `total_power` として時系列描画

## 実装ポイント（ファイル別）

- ボタン定義
  - [observer/observer/templates/qlook/index.html](observer/observer/templates/qlook/index.html#L26-L28)
- `total_power` モードでの要求送信
  - [observer/observer/static/index.js](observer/observer/static/index.js#L25-L31)
- トピック一覧の絞り込み（`quick_spectra`）
  - [observer/observer/server.py](observer/observer/server.py#L117-L127)
- フィールド選択UIと購読トグル
  - [observer/observer/static/quick-look.js](observer/observer/static/quick-look.js#L47-L66)
- 購読開始要求送信
  - [observer/observer/static/chart.js](observer/observer/static/chart.js#L56-L63)
- ROS購読作成と `ros2-message` 配信
  - [observer/observer/client_manager.py](observer/observer/client_manager.py#L106-L114)
  - [observer/observer/client_manager.py](observer/observer/client_manager.py#L130-L143)
- 総和計算と描画
  - [observer/observer/static/chart.js](observer/observer/static/chart.js#L165-L173)

## 計算式

配列データを `data[field] = [x_1, x_2, ..., x_n]` とすると、描画値は

$$
\mathrm{total\_power} = \sum_{i=1}^{n} x_i
$$

です。実装では `reduce` で合計しています。

## 補足

- `total_power` 分岐は「配列フィールド」の場合に有効です。
- `Graph` はID単位でインスタンスを共有するため、モード切替時のsocket扱いには注意が必要です。
