#include "timeDomainFilter.h"

/**
 * @brief 比较函数，用于qsort函数
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
 */
amplitudeLimitingFilter(float* getValue, float amp)
{
    uint16_t array_size = sizeof(getValue)/sizeof(float);
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
 * @param getValue 需要滤波的数组
 * @param N 采样窗口大小
 */
float medianValueFilter(float* getValue)
{
    uint16_t array_size = sizeof(getValue)/sizeof(float);
    uint16_t medianIndex = (array_size - 1) / 2;
    qsort(getValue, array_size, sizeof(float), cmp_float);
    return getValue[medianIndex]; // 返回排序后中位值
}

/**
 * @brief 算术平均滤波法
 * @param getValue 需要滤波的数组
*/
float arithmeticAverageFilter(float* getValue)
{
    uint16_t array_size = sizeof(getValue)/sizeof(float);
    float sum = 0;
    for (uint16_t i = 0; i < array_size; i++)
    {
        sum += getValue[i];
    }
    return sum / array_size; // 返回平均值
}

/**
 * @brief 几何平均滤波法
 * @param samples Input data array
 */
 float geometricMeanfilter(float* samples)
 {
     uint16_t array_size = sizeof(samples)/sizeof(float);
     if(array_size == 0) return 0;  // 新增空数组校验
     double product = 1.0;
     uint16_t valid_count = 0;
     
     for(uint16_t i = 0; i < array_size; i++)
     {
         if(samples[i] <= 0) continue; // 跳过非正值
         product *= samples[i];
         valid_count++;
     }
     
     if(valid_count == 0) return 0; // 全无效数据保护
     return pow(product, 1.0 / valid_count); // 修正为有效数据计数
 }
 
/**
 * @brief 一阶滞后滤波法
 * @param data_buffer Input data array
 * @param a Filter coefficient (0 ≤ a ≤ 1)
 * @param array_size Size of input array
 */
 float firstOrderLagFilter(float* data_buffer, float a)
 {
     uint16_t array_size = sizeof(data_buffer)/sizeof(float);
     if(a < 0 || a > 1) return 0;  // 参数校验
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
 */
float recursiveMovingAverageFilter(float* data_buffer, uint16_t window_size)
{
    float sum = 0;
    uint16_t array_size = sizeof(data_buffer)/sizeof(float);
    // 检查窗口大小是否有效
    if (window_size <= 0 || window_size > array_size)   
        return 0;
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
 */
float amplitudeLimitedMovingAverage(float* data_buffer, float amp, uint16_t window_size)
{
    uint16_t array_size = sizeof(data_buffer)/sizeof(float);
    if(array_size < window_size || window_size == 0) return 0;
    
    float sum = 0;
    uint16_t valid_count = 0;
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
 */
float debounce_filter(float* samples, uint16_t threshold)
{
    uint16_t array_size = sizeof(samples)/sizeof(float);
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
