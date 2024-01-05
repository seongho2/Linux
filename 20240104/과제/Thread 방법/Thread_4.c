#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <stdlib.h>

int sum1 = 0, sum2 = 0, sum3 = 0;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

void *func1(void *arg)
{
    for (int i = 1; i <= 100; i++)
    {
        pthread_mutex_lock(&mutex);
        sum1 += i;
        pthread_mutex_unlock(&mutex);
    }
    pthread_exit(NULL);
}

void *func2(void *arg)
{
    for (int i = 101; i <= 200; i++)
    {
        pthread_mutex_lock(&mutex);
        sum2 += i;
        pthread_mutex_unlock(&mutex);
    }
    pthread_exit(NULL);
}

void *func3(void *arg)
{
    for (int i = 201; i <= 300; i++)
    {
        pthread_mutex_lock(&mutex);
        sum3 += i;
        pthread_mutex_unlock(&mutex);
    }
    pthread_exit(NULL);
}

int main()
{
    pthread_t tid1, tid2, tid3;

    if (pthread_create(&tid1, NULL, func1, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, func2, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, func3, NULL) != 0)
    {
        fprintf(stderr, "thread create error\n");
        exit(1);
    }

    pthread_join(tid1, NULL);
    pthread_join(tid2, NULL);
    pthread_join(tid3, NULL);

    int result = sum1 + sum2 + sum3;

    printf("Total Sum: %d\n", result);

    return 0;
}
