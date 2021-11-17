#include <Arduino.h>
#include "SUtils.h"

void printQueueUsageInfo(QueueHandle_t xQueue, const char* readableQueueName)
{
    int messagesWaiting = uxQueueMessagesWaiting(xQueue);

    Serial.print(readableQueueName);
    Serial.println(" usage: ");
    
    Serial.println("Number of messages waiting: ");
    Serial.print(messagesWaiting);
    
    Serial.println("Max possible number of waiting messages: ");
    Serial.print(uxQueueSpacesAvailable(xQueue) + messagesWaiting);
}

void printStackUsageInfo(const char* readableTaskName)
{
    // Print out remaining stack memory (in words of 4 bytes)
    Serial.print(readableTaskName);
    Serial.println(" high water mark (words): ");
    Serial.println(uxTaskGetStackHighWaterMark(NULL));
} 