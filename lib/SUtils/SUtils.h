#ifndef S_UTILS_H /*== INCLUDE ==*/
#define S_UTILS_H /*=== GUARD ===*/
typedef float float32_t;

void printQueueUsageInfo(QueueHandle_t xQueue, const char* readableQueueName);
void printStackUsageInfo(const char* readableTaskName);
void printStringAndFloat(char* s, float32_t f);
#endif // S_UTILS_H -- End of the header file