#include "timeDomainFilter.h"

/**
 * @brief 比较函数，用于qsort函数调用
 * @param e1 第一个元素
 * @param e2 第二个元素
 * @return 比较结果
 */
int cmp_float(const void* e1, const void* e2)
{
	if (*(float*)e1 == *(float*)e2)
		return 0;
	else if (*(float*)e1 > *(float*)e2)
		return 1;
	else
		return -1;
}

/**
 * @brief 限幅滤波法（又称程序判断滤波法）
 * @param getValue 需要滤波的数组
 * @param amp 限幅值
 * @param array_size 数组大小
 * @note 优点:能有效克服因偶然因素引起的脉冲干扰
 * @note 缺点:无法抑制那种周期性的干扰,平滑度差
 */
void amplitudeLimitingFilter(float* getValue, float amp, uint16_t array_size)
{
    float lastValue = getValue[0];
    for (uint16_t i = 0; i < array_size; i++)
    {
        if (abs(getValue[i] - lastValue) > amp)
            getValue[i] = amp;
        if (i != 0)
            lastValue = getValue[i-1];
    }
}

/**
 * @brief 中位值滤波法
 * @param samples Input data array
 * @param array_size Size of input array
 * @note 优点：能有效克服因偶然因素引起的波动干扰，对温度、液位的变化缓慢的被测参数有良好的滤波效果
 * @note 缺点：对流量、速度等快速变化的参数不宜
 */
 float medianValueFilter(float* samples, uint16_t array_size)
 {
     if(array_size == 0) return NAN;  // 新增空数组校验
     
     // 创建临时数组避免修改原始数据
     float* temp = (float*)malloc(array_size * sizeof(float));
     memcpy(temp, samples, array_size * sizeof(float));
     
     qsort(temp, array_size, sizeof(float), cmp_float);
     uint16_t median_index = (array_size % 2 == 0) ? 
                           (array_size/2 - 1) :    // 偶数取前中位数
                           (array_size - 1)/2;      // 奇数取中位数
     
     float result = temp[median_index];
     free(temp);
     return result;
 }
 
/**
 * @brief 算术平均滤波法
 * @param samples Input data array
 * @param array_size Valid data length (must > 0)
 * @return Filtered average value (NAN when invalid)
 * @note 优点：适用于对一般具有随机干扰的信号进行滤波,这样信号的特点是有一个平均值，信号在某一数值范围附近上下波动
 * @note 缺点：对于测量速度较慢或要求数据计算速度较快的实时控制不适用,比较浪费RAM 
 */
 float arithmeticAverageFilter(float* samples, uint16_t array_size)
 {
     if(array_size == 0 || samples == NULL) return NAN;
     
     double sum = 0.0;
     for(uint16_t i = 0; i < array_size; i++)
     {
         sum += (double)samples[i];
     }
     return (float)(sum / array_size); // 最终结果转回float
 }

/**
 * @brief 几何平均滤波法
 * @param samples Input data array
 * @param array_size Valid data length (must > 0)
 * @note 优点：
            1. 抗干扰能力强：对异常值不敏感，能有效抑制少数极端数据点的影响
            2. 适合特定数据分布：对呈几何级数或对数分布的数据（如光强、声压等）能提供更准确的趋势反映
            4. 最适合用于处理呈指数特征变化的传感器数据
            5. 适用于对异常值敏感但实时性要求不高的监测系统
 * @note 缺点：
            1. 计算复杂度高：涉及乘法和开方运算，消耗较多CPU资源
            2. 数值限制：不能处理零值和负值，否则会导致结果无效
            3. 实现难度大：在定点数系统中容易溢出，且引入量化误差
            4. 实时性差：相比简单滤波算法需要更长的计算时间
 */
 float geometricMeanFilter(float* samples, uint16_t array_size)
 {
     if(array_size == 0) return NAN;  // 新增空数组校验
     double product = 1.0;
     uint16_t valid_count = 0;
     
     for(uint16_t i = 0; i < array_size; i++)
     {
         if(samples[i] <= 0) continue; // 跳过非正值
         product *= samples[i];
         valid_count++;
     }
     
     if(valid_count == 0) return NAN; // 全无效数据保护
     return pow(product, 1.0 / valid_count);
 }
 
/**
 * @brief 一阶滞后滤波法
 * @param data_buffer Input data array
 * @param a Filter coefficient (0 ≤ a ≤ 1)
 * @param array_size Size of input array
 * @note 优点：对周期性干扰具有良好的抑制作用，适用于波动频率较高的场合
 * @note 缺点：相位滞后，灵敏度低; 滞后程度取决于a值大小; 不能消除滤波频率高于采样频率的1/2的干扰信号
 */
 float firstOrderLagFilter(float* data_buffer, float a, uint16_t array_size)
 {
     if(a < 0 || a > 1) return NAN;  // 参数校验
     float filtered_value = data_buffer[0];
     
     for(uint16_t i = 1; i < array_size; i++)  // 从第二个元素开始
     {
         filtered_value = a * data_buffer[i] + (1 - a) * filtered_value;
     }
     return filtered_value;
 }

/**
 * @brief 递推平均滤波算法 (Sliding window implementation)
 * @param data_buffer Pointer to data array
 * @param window_size Size of moving window
 * @param array_size Total size of input array
 * @note 优点：对周期性干扰有良好的抑制作用，平滑度高;适用于高频振荡的系统
 * @note 缺点：灵敏度低,对偶然出现的脉冲性干扰的抑制作用较差,不易消除由于脉冲干扰所引起的采样值偏差,不适用于脉冲干扰比较严重的场合,比较浪费RAM
 */
float recursiveMovingAverageFilter(float* data_buffer, uint16_t window_size, uint16_t array_size)
{
    float sum = 0;
    // 检查窗口大小是否有效
    if (window_size <= 0 || window_size > array_size)   
        return NAN;
    // 初始化窗口求和
    for(uint16_t i=0; i<window_size; i++){
        sum += data_buffer[i];
    }
    // 滑动窗口处理
    for(uint16_t i=window_size; i<array_size; i++)
    {
        sum = sum - data_buffer[i - window_size] + data_buffer[i];
        data_buffer[i - window_size] = sum / window_size;
    }
    // 返回最后一个有效滤波值
    return data_buffer[array_size - window_size];
}

/**
 * @brief 限幅平均滤波法
 * @param data_buffer Input data array
 * @param amp Amplitude limit value
 * @param window_size Moving average window size
 * @param array_size Total size of input array
 * @note 优点：融合了两种滤波法的优点,对于偶然出现的脉冲性干扰，可消除由于脉冲干扰所引起的采样值偏差
 * @note 缺点：比较浪费RAM
 */
float amplitudeLimitedMovingAverage(float* data_buffer, float amp, uint16_t window_size, uint16_t array_size)
{
    if(array_size < window_size || window_size == 0) return NAN;
    
    float sum = 0;
    float prev_value = data_buffer[0];
    
    // 限幅预处理
    for(uint16_t i = 0; i < array_size; i++){
        float diff = fabs(data_buffer[i] - prev_value);
        if(diff > amp) {
            data_buffer[i] = prev_value + ((data_buffer[i] > prev_value) ? amp : -amp);
        }
        prev_value = data_buffer[i];
    }
    
    // 滑动平均计算
    for(uint16_t i = 0; i < window_size; i++){
        sum += data_buffer[i];
    }
    
    float min_sum = sum;
    for(uint16_t i = window_size; i < array_size; i++){
        sum += data_buffer[i] - data_buffer[i - window_size];
        if(sum < min_sum) min_sum = sum;
    }
    
    return min_sum / window_size;
}

/**
 * @brief 消抖滤波法
 * @param samples Input data array
 * @param threshold Counter threshold (N)
 * @param array_size Size of input array
 * @note 优点：对于变化缓慢的被测参数有较好的滤波效果，可避免在临界值附近控制器的反复开/关跳动或显示器上数值抖动
 * @note 缺点：对于快速变化的参数不宜,如果在计数器溢出的那一次采样到的值恰好是干扰值，则会将干扰值当作有效值导入系统
 */
float debounceFilter(float* samples, uint16_t threshold, uint16_t array_size)
{
    if(array_size == 0 || threshold == 0) return NAN;
    
    static uint16_t counter = 0;      // 消抖计数器
    static float current_value = 0;   // 当前有效值
    current_value = samples[0];       // 初始化为第一个采样值
    
    for(uint16_t i = 0; i < array_size; i++){
        if(samples[i] == current_value){
            counter = 0;             // 采样值有效，清零计数器
        } else {
            counter++;               // 采样值变化，增加计数器
            if(counter >= threshold){
                current_value = samples[i];  // 达到阈值更新有效值
                counter = 0;         // 重置计数器
            }
        }
    }
    return current_value;
}
