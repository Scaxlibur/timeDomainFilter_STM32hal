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