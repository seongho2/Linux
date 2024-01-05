#include <stdio.h>
#include <pthread.h>

#define MAX_NUMBER 300

int shared_sum = 0;
pthread_mutex_t mutex;

void* thread_function(void* arg) 
{
    int start = *((int*)arg);

    for (int i = start; i <= MAX_NUMBER; i += 2) 
    {
        pthread_mutex_lock(&mutex);
        shared_sum += i;
        pthread_mutex_unlock(&mutex);
    }

    pthread_exit(NULL);
}

int main() 
{
    if (pthread_mutex_init(&mutex, NULL) != 0) 
    {
        fprintf(stderr, "Mutex 초기화에 실패했습니다.\n");
        return 1;
    }

    pthread_t thread1, thread2;

    int start1 = 1;
    int start2 = 2;

    pthread_create(&thread1, NULL, thread_function, &start1);
    pthread_create(&thread2, NULL, thread_function, &start2);

    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);

    printf("1부터 %d 까지 더한 값은 : %d\n", MAX_NUMBER, shared_sum);

    return 0;
}
