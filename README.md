# GNSS信号捕获过程分析与参数对比

## 1. 信号捕获过程

GNSS信号捕获是接收机处理的第一步，目的是确定哪些卫星可见并估计其初始参数。信号捕获过程主要采用并行码相位搜索算法，通过FFT加速实现。

### 1.1 捕获主要步骤

信号捕获通过以下主要步骤实现：

1. **信号预处理**：
   - 去除直流偏置：`signal0DC = longSignal - mean(longSignal);`
   - 分离信号为两个1ms段用于处理：
     ```matlab
     signal1 = longSignal(1 : samplesPerCode);
     signal2 = longSignal(samplesPerCode+1 : 2*samplesPerCode);
     ```

2. **PRN码准备**：
   - 为所有卫星生成CA码表：`caCodesTable = makeCaTable(settings);`
   - 转换到频域：`caCodeFreqDom = conj(fft(caCodesTable(PRN, :)));`

3. **频率搜索**：
   - 基于中频和搜索带宽创建频率网格：
     ```matlab
     frqBins(frqBinIndex) = settings.IF - settings.acqSearchBand + 0.5e3 * (frqBinIndex - 1);
     ```
   - 每500Hz一个搜索点

4. **相关计算**：
   - 下变频处理：
     ```matlab
     sigCarr = exp(1i*frqBins(frqBinIndex) * phasePoints);
     I1 = real(sigCarr .* signal1);
     Q1 = imag(sigCarr .* signal1);
     ```
   - 频域相乘实现时域相关：
     ```matlab
     IQfreqDom1 = fft(I1 + 1i*Q1);
     convCodeIQ1 = IQfreqDom1 .* caCodeFreqDom;
     acqRes1 = abs(ifft(convCodeIQ1));
     ```

5. **信号检测**：
   - 寻找最大相关峰：`[peakSize, codePhase] = max(max(results));`
   - 寻找次大峰值并计算比值：
     ```matlab
     secondPeakSize = max(results(frequencyBinIndex, codePhaseRange));
     acqResults.peakMetric(PRN) = peakSize/secondPeakSize;
     ```
   - 基于阈值判决：`if (peakSize/secondPeakSize) > settings.acqThreshold`

6. **精细频率估计**：
   对检测到的信号进行精细频率估计，通过去除码调制后FFT实现：
   ```matlab
   fftxc = abs(fft(xCarrier, fftNumPts));
   [~, fftMaxIndex] = max(fftxc);
   ```

## 2. Urban与Opensky场景参数分析

两种场景下的共同捕获参数包括：

| 参数        | 值          | 说明                     |
| ----------- | ----------- | ------------------------ |
| 卫星PRN范围 | 1:32        | 搜索所有GPS卫星          |
| CA码长度    | 1023 chips  | 标准GPS C/A码长度        |
| CA码基频    | 1.023 MHz   | 标准GPS C/A码频率        |
| 检测阈值    | 1.5         | 峰值比阈值，判定信号存在 |
| 相关器间隔  | 0.5 chip    | DLL相关器间隔            |
| DLL阻尼比   | 0.707       | 码跟踪环路阻尼比         |
| 搜索步长    | 500 Hz      | 频率搜索网格间隔         |
| 搜索算法    | FFT并行搜索 | 加速相关计算             |

两种场景有以下关键差异：

| 参数         | Urban场景 | Opensky场景 | 影响分析                                             |
| ------------ | --------- | ----------- | ---------------------------------------------------- |
| 中频(IF)     | 0 Hz      | 4.58 MHz    | Urban采用零中频采集，处理更简单；Opensky需中频下变频 |
| 采样频率     | 26 MHz    | 58 MHz      | Opensky有更高采样率，可提供更精确的码相位估计        |
| 捕获搜索带宽 | ±8 kHz    | ±7 kHz      | Urban考虑更大多普勒范围，适合城市高动态环境          |
| PLL噪声带宽  | 18 Hz     | 20 Hz       | Opensky使用更宽PLL带宽，可能更适应频率变化           |
| 数据文件     | Urban.dat | Opensky.bin | 不同数据格式，可能有不同前端处理                     |

## 3. 参数设置影响分析

1. **采样率影响**：
   - 更高采样率提供更精细的码相位分辨率
   - 采样率与处理复杂度成正比

2. **中频设置影响**：
   - 零中频设计简化了处理，但可能受DC偏置影响
   - 非零中频需下变频，但可避免低频干扰

3. **搜索带宽影响**：
   - 更宽搜索带宽可捕获更高动态环境下的信号
   - 但会增加搜索空间和虚警概率

4. **阈值设置影响**：
   - 较低阈值可增加弱信号检测概率
   - 但会增加虚警率

## 4 捕获结果可视化

捕获结果通过条形图直观显示：
- Y轴表示捕获度量（Peak Metric）：最大相关峰值与次大相关峰值的比值
- 绿色柱状表示成功捕获的卫星
- 蓝色柱状表示未能捕获的卫星


捕获性能指标

信号捕获性能可以通过以下指标评估：

1. **捕获成功率**：成功捕获的卫星数量与总卫星数的比值
2. **捕获度量（Peak Metric）**：最大相关峰与次大相关峰的比值，反映信噪比
3. **频率估计精度**：估计的载波频率与真实频率的差异
4. **码相位估计精度**：估计的码相位与真实码相位的差异

通过绘制捕获结果条形图，可以直观比较不同场景下的信号强度和捕获性能。Urban环境通常信号较弱且多径严重，而Opensky环境信号较强且多径较少。

