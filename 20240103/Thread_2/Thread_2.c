#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

void *firstThreadRun(void *arg)
{
    int table = *((int *)arg);

    for (int i = 1; i <= 9; ++i)
    {
        printf("%d x %d = %d\n", table, i, table * i);
        sleep(1);
    }

    return NULL;
}

void *secondThreadRun(void *arg)
{
    int table = *((int *)arg);

    for (int i = 1; i <= 9; ++i)
    {
        printf("%d x %d = %d\n", table, i, table * i);
        sleep(1);
    }

    return NULL;
}

void *thirdThreadRun(void *arg)
{
    int table = *((int *)arg);

    for (int i = 1; i <= 9; ++i)
    {
        printf("%d x %d = %d\n", table, i, table * i);
        sleep(1);
    }

    return NULL;
}

int main()
{
    pthread_t thread1, thread2, thread3;

    int table1 = 1, table2 = 2, table3 = 3;

    pthread_create(&thread1, NULL, firstThreadRun, (void *)&table1);
    pthread_join(thread1, NULL); 

    pthread_create(&thread2, NULL, secondThreadRun, (void *)&table2);
    pthread_join(thread2, NULL);

    pthread_create(&thread3, NULL, thirdThreadRun, (void *)&table3);
    pthread_join(thread3, NULL);

    return 0;
}
