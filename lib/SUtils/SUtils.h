#ifndef S_UTILS_H /*== INCLUDE ==*/
#define S_UTILS_H /*=== GUARD ===*/

void printQueueUsageInfo(QueueHandle_t xQueue, const char* readableQueueName);
void printStackUsageInfo(const char* readableTaskName);

#endif // S_UTILS_H -- End of the header file