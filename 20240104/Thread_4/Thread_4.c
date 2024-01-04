#include <stdio.h>
#include <pthread.h>
#include <stdlib.h>

typedef struct {
    int start;
    int end;
} Range;

void* sumInRange(void* arg) {
    Range* range = (Range*)arg;
    int sum = 0;

    for (int i = range->start; i <= range->end; ++i) {
        sum += i;
    }

    printf("%d~%d의 합: %d\n", range->start, range->end, sum);

    return (void*)sum;
}

int calculateTotalSum(int sum1, int sum2, int sum3) {
    return sum1 + sum2 + sum3;
}

int main() {
    pthread_t tid1, tid2, tid3;
    int sum1, sum2, sum3;

    Range range1 = {0, 100};
    Range range2 = {101, 200};
    Range range3 = {201, 300};

    if (pthread_create(&tid1, NULL, sumInRange, (void*)&range1) != 0 ||
        pthread_create(&tid2, NULL, sumInRange, (void*)&range2) != 0 ||
        pthread_create(&tid3, NULL, sumInRange, (void*)&range3) != 0) {
        fprintf(stderr, "스레드 생성 실패\n");
        exit(1);
    }

    pthread_join(tid1, (void**)&sum1);
    pthread_join(tid2, (void**)&sum2);
    pthread_join(tid3, (void**)&sum3);

    int totalSum = calculateTotalSum(sum1, sum2, sum3);
    printf("총합: %d\n", totalSum);

    return 0;
}
