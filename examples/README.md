# StampFly ESKF Estimator - Examples

このディレクトリには、StampFly ESKF Estimatorの使用方法を示すサンプルコードが含まれています。

## 📋 利用可能な例

### simple_estimation_eskf.cpp

基本的なESKF推定器の使用方法を示す最小限の例です。

**内容:**
- パラメータの作成と設定
- 推定器の初期化
- シミュレートされたセンサーデータの処理
- 状態の取得と表示

**学習内容:**
- `ESKFEstimator` クラスの基本的な使い方
- IMU、地磁気、気圧、ToF、オプティカルフローデータの処理
- 異なるセンサーレートでの非同期更新
- 推定状態の取得方法

## 🔧 ビルド方法

### 前提条件

これらの例は、本番実装（`src/`ディレクトリ）が完成した後にのみビルド・実行可能です。

現在は **APIデモンストレーション** として提供されています。

### ビルドコマンド

```bash
# プロジェクトルートから
mkdir build && cd build
cmake -DBUILD_EXAMPLES=ON ..
make

# 例の実行
./examples/simple_estimation_eskf
```

## 📝 サンプルコードの構造

各例は以下の流れで構成されています：

1. **パラメータ設定**: `ESKFParameters` の作成
2. **推定器の初期化**: `ESKFEstimator` のインスタンス化と初期状態設定
3. **センサーデータ処理ループ**:
   - `processImu()` - 高頻度（200Hz）
   - `processMagnetometer()` - 中頻度（50Hz）
   - `processBarometer()`, `processTof()` - 低頻度（20Hz）
   - `processOpticalFlow()` - 中頻度（30Hz）
4. **状態取得**: `getState()`, `getPosition()`, `getVelocity()` など
5. **診断情報**: ステータス、棄却された測定値の数など

## 🎓 実装ガイド

### 本番システムへの統合手順

1. **センサープロバイダーの実装**
   ```cpp
   class MyImuProvider : public IImuProvider {
       // ハードウェア固有の実装
   };
   ```

2. **センサースイートの作成**
   ```cpp
   auto sensors = std::make_shared<SensorSuite>();
   sensors->setImuProvider(std::make_shared<MyImuProvider>());
   // ... 他のセンサーも設定
   ```

3. **推定器への接続**
   ```cpp
   estimator.connectSensorSuite(sensors);
   ```

4. **コールバック駆動モード**
   センサースイートがコールバックを自動的に推定器に送信します。

## 🔍 詳細な実装例

より高度な実装例については、以下を参照してください：
- リファレンス実装: `docs/reference_impl/cpp/`
- 理論ドキュメント: `docs/theory/ESKF_Position_Estimation.md`

## ⚠️ 注意事項

- これらの例はシミュレートされたデータを使用しています
- 実機での使用前に、実際のセンサーノイズ特性に合わせてパラメータを調整してください
- 安全のため、フェイルセーフ機能を必ず実装してください

## 🤝 貢献

新しいサンプルコードの追加や既存コードの改善は歓迎します！
Pull Requestをお送りください。
