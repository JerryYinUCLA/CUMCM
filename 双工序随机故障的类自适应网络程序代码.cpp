#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

const int DISTANCE[4] = {0, 23, 41, 59};
const int SERVINGTIME[2] = {30, 35};
const int WORK[2] = {280, 500};
const int WASH = 30;
const double RATE = 0.005;

int knifeNum();
char* knifeDistribution(int numA);
double* Simulation(double alpha[8], double beta[8], double kappa[8], bool last);
double* manySimulations(int num);
int maxIndex(double a[8]);
int minIndex(double a[8]);
int* topBottomIndices(double a[8]);
double power(double base, int exponent);
int computeProb(int num);

int main()
{
    double* final = manySimulations(1000);
    for (int i=0; i<8; i++)
    {
        cout << "CNC";
        cout << i + 1;
        cout << ": alpha = ";
        cout << final[i];
        cout << " beta = ";
        cout << final[i+8];
        cout << " kappa = ";
        cout << final[i+16] << endl;
    }
}

int knifeNum()
{
    int Aodd, Aeven;
    double diff[16];
    for (Aodd = 1; Aodd <= 4; Aodd++)
    {
        for (Aeven = 0; Aeven <= 3; Aeven++)
        {
            int Bodd = 4 - Aodd;
            int Beven = 4 - Aeven;
            double rate1 = double(WORK[0] + double(Aodd * SERVINGTIME[0] + Aeven * SERVINGTIME[1]) / (Aodd + Aeven)) / (WORK[1] + WASH + double(Bodd * SERVINGTIME[0] + Beven * SERVINGTIME[1]) / (Bodd + Beven));
            double rate2 = double(Aodd + Aeven) / (Bodd + Beven);
            diff[4 * (Aodd-1) + Aeven] = abs(rate1 - rate2);
        }
    }
    int numA = minIndex(diff) / 4 + minIndex(diff) % 4 + 1;
    return numA;
}

char* knifeDistribution(int numA)
{
    static char distribute[8];
    for (int i=0; i<8; i++)
        distribute[i] = 'B';
    switch (numA)
    {
        case 1:
            distribute[2] = 'A';
            break;
        case 2:
            distribute[2] = 'A';
            distribute[4] = 'A';
            break;
        case 3:
            distribute[2] = 'A';
            distribute[4] = 'A';
            distribute[0] = 'A';
            break;
        case 4:
            distribute[6] = 'A';
            distribute[2] = 'A';
            distribute[4] = 'A';
            distribute[0] = 'A';
            break;
        case 5:
            distribute[1] = 'A';
            distribute[6] = 'A';
            distribute[2] = 'A';
            distribute[4] = 'A';
            distribute[0] = 'A';
            break;
        case 6:
            distribute[7] = 'A';
            distribute[1] = 'A';
            distribute[6] = 'A';
            distribute[2] = 'A';
            distribute[4] = 'A';
            distribute[0] = 'A';
            break;
        default:
            distribute[3] = 'A';
            distribute[7] = 'A';
            distribute[1] = 'A';
            distribute[6] = 'A';
            distribute[2] = 'A';
            distribute[4] = 'A';
            distribute[0] = 'A';
            break;
    }
    return distribute;
}

double* Simulation(double alpha[8], double beta[8], double kappa[8], bool last)
{
    char* knives = knifeDistribution(knifeNum());
    int waitTime[8], currentDist[8], cumuWait[8], cumuDist[8], workTime[8], counter[8], breakTime[8], broken[8], wastedTime[8], brokenNums[8];
    double cumuCost[8];
    bool done[8], firstTime[8];
    for (int i=0; i<8; i++)
    {
        waitTime[i] = 0;
        currentDist[i] = DISTANCE[i / 2];
        cumuWait[i] = 0;
        cumuDist[i] = 0;
        workTime[i] = 0;
        counter[i] = 0;
        cumuCost[i] = 0;
        done[i] = true;
        firstTime[i] = true;
        breakTime[i] = 0;
        broken[i] = 0;
        brokenNums[i] = 0;
        wastedTime[i] = 0;
    }
    double cost[8];
    // The initial location of the RGV
    int locRGV = 0;
    int num = 0;
    int numA = 0;
    int numB = 0;
    int duration = 0;
    int arrived = 0;
    int nextToServe = 0;
    int move = 0;
    int temp = 0;
    for (int time = 0; time<28800; time++)
    {
        if (time != 0)
        {
            arrived++;
            if (arrived <= temp)
            {
                for (int i=0; i<8; i++)
                {
                    if (done[i])
                        waitTime[i]++;
                    else if (breakTime[i] != 0)
                        breakTime[i]--;
                    else
                    {
                        if (workTime[i] == 0)
                        {
                            done[i] = true;
                            waitTime[i] = 0;
                            firstTime[i] = true;
                        }
                        else
                        {
                            if (knives[i] == 'A')
                            {
                                broken[i] = rand() % computeProb(WORK[0]);
                                if (broken[i] == 0)
                                {
                                    breakTime[i] = rand() % 601 + 600;
                                    wastedTime[i] += breakTime[i] + WORK[0] - workTime[i];
                                    numA--;
                                    brokenNums[i]++;
                                    workTime[i] = 0;
                                }
                                else
                                {
                                    workTime[i]--;
                                    if (workTime[i] == 0)
                                    {
                                        done[i] = true;
                                        waitTime[i] = 0;
                                    }
                                }
                            }
                            if (knives[i] == 'B')
                            {
                                broken[i] = rand() % computeProb(WORK[1]);
                                if (broken[i] == 0)
                                {
                                    breakTime[i] = rand() % 601 + 600;
                                    wastedTime[i] += breakTime[i] + WORK[1] - workTime[i];
                                    numB--;
                                    brokenNums[i]++;
                                    workTime[i] = 0;
                                }
                                else
                                {
                                    workTime[i]--;
                                    if (workTime[i] == 0)
                                    {
                                        done[i] = true;
                                        waitTime[i] = 0;
                                    }
                                }
                            }
                        }
                    }
                }
                if (breakTime[nextToServe] != 0)
                {
                    duration = 0;
                    num--;
                }
            }
            else if (arrived < temp + SERVINGTIME[nextToServe % 2])
            {
                for (int i=0; i<8; i++)
                {
                    if (i != nextToServe)
                    {
                        if (done[i])
                            waitTime[i]++;
                        else if (breakTime[i] != 0)
                            breakTime[i]--;
                        else
                        {
                            if (workTime[i] == 0)
                            {
                                done[i] = true;
                                waitTime[i] = 0;
                                firstTime[i] = true;
                            }
                            else
                            {
                                if (knives[i] == 'A')
                                {
                                    broken[i] = rand() % computeProb(WORK[0]);
                                    if (broken[i] == 0)
                                    {
                                        breakTime[i] = rand() % 601 + 600;
                                        wastedTime[i] += breakTime[i] + WORK[0] - workTime[i];
                                        numA--;
                                        brokenNums[i]++;
                                        workTime[i] = 0;
                                    }
                                    else
                                    {
                                        workTime[i]--;
                                        if (workTime[i] == 0)
                                        {
                                            done[i] = true;
                                            waitTime[i] = 0;
                                        }
                                    }
                                }
                                if (knives[i] == 'B')
                                {
                                    broken[i] = rand() % computeProb(WORK[1]);
                                    if (broken[i] == 0)
                                    {
                                        breakTime[i] = rand() % 601 + 600;
                                        wastedTime[i] += breakTime[i] + WORK[1] - workTime[i];
                                        numB--;
                                        brokenNums[i]++;
                                        workTime[i] = 0;
                                    }
                                    else
                                    {
                                        workTime[i]--;
                                        if (workTime[i] == 0)
                                        {
                                            done[i] = true;
                                            waitTime[i] = 0;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            else if (arrived == temp + SERVINGTIME[nextToServe % 2])
            {
                waitTime[nextToServe] = 0;
                workTime[nextToServe] = WORK[(num + 1) % 2];
                done[nextToServe] = false;
                for (int i=0; i<8; i++)
                {
                    if (done[i])
                        waitTime[i]++;
                    else if (breakTime[i] != 0)
                        breakTime[i]--;
                    else
                    {
                        if (workTime[i] == 0)
                        {
                            done[i] = true;
                            waitTime[i] = 0;
                            firstTime[i] = true;
                        }
                        else
                        {
                            if (knives[i] == 'A')
                            {
                                broken[i] = rand() % computeProb(WORK[0]);
                                if (broken[i] == 0)
                                {
                                    breakTime[i] = rand() % 601 + 600;
                                    wastedTime[i] += breakTime[i] + WORK[0] - workTime[i];
                                    numA--;
                                    brokenNums[i]++;
                                    workTime[i] = 0;
                                }
                                else
                                {
                                    workTime[i]--;
                                    if (workTime[i] == 0)
                                    {
                                        done[i] = true;
                                        waitTime[i] = 0;
                                    }
                                }
                            }
                            if (knives[i] == 'B')
                            {
                                broken[i] = rand() % computeProb(WORK[1]);
                                if (broken[i] == 0)
                                {
                                    breakTime[i] = rand() % 601 + 600;
                                    wastedTime[i] += breakTime[i] + WORK[1] - workTime[i];
                                    numB--;
                                    brokenNums[i]++;
                                    workTime[i] = 0;
                                }
                                else
                                {
                                    workTime[i]--;
                                    if (workTime[i] == 0)
                                    {
                                        done[i] = true;
                                        waitTime[i] = 0;
                                    }
                                }
                            }
                        }
                    }
                }
                arrived = -1000000;
            }
        }
        if (duration != 0)
        {
            duration--;
            continue;
        }
        num++; // To see whether the RGV should serve A or B
        bool hasReadyA, hasReadyB, readyA[8], readyB[8];
        hasReadyA = false;
        hasReadyB = false;
        for (int i=0; i<8; i++)
        {
            readyA[i] = done[i] && (knives[i] == 'A');
            if (readyA[i])
                hasReadyA = true;
            readyB[i] = done[i] && (knives[i] == 'B');
            if (readyB[i])
                hasReadyB = true;
        }
        if (num % 2 == 1)
        {
            numA++;
            if (hasReadyA)
            {
                for (int i=0; i<8; i++)
                {
                    if (readyA[i])
                    {
                        cost[i] = alpha[i] * waitTime[i] - beta[i] * currentDist[i] + kappa[i];
                        counter[i]++;
                        cumuCost[i] += cost[i];
                    }
                    else
                        cost[i] = -10000000;
                }
                // Choose a machine A to serve
                nextToServe = maxIndex(cost);
                // The number of steps needed to take
                move = abs(nextToServe / 2 - locRGV / 2);
                // Total time the RGV needs to serve the chosen CNC, including moving, uploading and maybe washing
                duration = DISTANCE[move] + SERVINGTIME[nextToServe % 2];
                cumuWait[nextToServe] += waitTime[nextToServe] + DISTANCE[move];
                // For each CNC A, accumulate its distance from the RGV
                for (int i=0; i<8; i++)
                {
                    if (knives[i] == 'A')
                        cumuDist[i] += currentDist[i];
                }
                // Update each CNC's distance from the RGV
                for (int i=0; i<8; i++)
                    currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
                // Update the position of the RGV
                locRGV = nextToServe;
                // Update the time counter
                duration--;
                arrived = 0;
                temp = DISTANCE[move];
            }
            else
            {
                for (int i=0; i<8; i++)
                {
                    if (knives[i] == 'A')
                        cost[i] = max(DISTANCE[abs(locRGV / 2 - i / 2)], workTime[i]);
                    else
                        cost[i] = 10000000000;
                }
                nextToServe = minIndex(cost);
                duration = cost[nextToServe] + SERVINGTIME[nextToServe % 2];
                // Update each CNC's distance from the RGV
                for (int i=0; i<8; i++)
                    currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
                // Update the position of the RGV
                locRGV = nextToServe;
                // Update the time counter
                duration--;
                arrived = 0;
                temp = cost[nextToServe];
            }
        }
        else
        {
            numB++;
            if (hasReadyB)
            {
                for (int i=0; i<8; i++)
                {
                    if (readyB[i])
                    {
                        cost[i] = alpha[i] * waitTime[i] - beta[i] * currentDist[i] + kappa[i];
                        counter[i]++;
                        cumuCost[i] += cost[i];
                    }
                    else
                        cost[i] = -10000000;
                }
                // Choose a machine B to serve
                nextToServe = maxIndex(cost);
                // The number of steps needed to take
                move = abs(nextToServe / 2 - locRGV / 2);
                // Total time the RGV needs to serve the chosen CNC, including moving, uploading and maybe washing
                if (firstTime[nextToServe])
                {
                    duration = DISTANCE[move] + SERVINGTIME[nextToServe % 2];
                    cumuWait[nextToServe] += waitTime[nextToServe] + DISTANCE[move];
                    firstTime[nextToServe] = false;
                }
                else
                {
                    duration = DISTANCE[move] + SERVINGTIME[nextToServe % 2] + WASH;
                    cumuWait[nextToServe] += waitTime[nextToServe] + DISTANCE[move];
                }
                // For each CNC B, accumulate its distance from the RGV
                for (int i=0; i<8; i++)
                {
                    if (knives[i] == 'B')
                        cumuDist[i] += currentDist[i];
                }
                // Update each CNC's distance from the RGV
                for (int i=0; i<8; i++)
                    currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
                // Update the position of the RGV
                locRGV = nextToServe;
                // Update the time counter
                duration--;
                arrived = 0;
                temp = DISTANCE[move];
            }
            else
            {
                for (int i=0; i<8; i++)
                {
                    if (knives[i] == 'B')
                        cost[i] = max(DISTANCE[abs(locRGV / 2 - i / 2)], workTime[i]);
                    else
                        cost[i] = 10000000000;
                }
                nextToServe = minIndex(cost);
                duration = cost[minIndex(cost)] + SERVINGTIME[nextToServe % 2];
                if (firstTime[nextToServe])
                    duration += WASH;
                // Update each CNC's distance from the RGV
                for (int i=0; i<8; i++)
                    currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
                // Update the position of the RGV
                locRGV = nextToServe;
                // Update the time counter
                duration--;
                arrived = 0;
                temp = cost[nextToServe];
            }
        }
    }
    static double results[24];
    for (int i=0; i<8; i++)
    {
        results[i] = cumuCost[i] / counter[i];
        results[i+8] = cumuWait[i];
        results[i+16] = cumuDist[i];
    }
    if (last)
    {
        for (int i=8; i<16; i++)
        {
            cout << "CNC";
            cout << i - 7;
            cout << " Total Waiting Time: ";
            cout << results[i] << endl;
        }
        cout << numB << endl;
    }
    return results;
}

double* manySimulations(int num)
{
    double alpha[8], beta[8], kappa[8];
    // Initialize the coefficients randomly
    for (int i=0; i<8; i++)
    {
        alpha[i] = (rand() % 5000000 + 1) / double(5000001);
        beta[i] = (rand() % 5000000 + 1) / double(5000001);
        kappa[i] = 0;
    }
    double M[7][7];
    for (int i=0; i<7; i++)
    {
        for (int j=0; j<7; j++)
        {
            if (j == 0)
                M[i][j] = 0;
            else if (i - j >= 1)
                M[i][j] = 0;
            else if (i == 0)
                M[i][j] = 1;
            else if (i == j)
                M[i][j] = -1;
            else if ((i == 2 && j == 3) || (i == 3 && j == 4))
                M[i][j] = -0.5;
            else if ((i == 1 && j == 3) || (i == 1 && j == 4))
                M[i][j] = 0.5;
            else if ((i == 1 && j == 5) || (i == 1 && j == 6))
                M[i][j] = 0.67;
            else if ((i == 2 && j == 5) || (i == 2 && j == 6))
                M[i][j] = 0.33;
            else if ((i == 3 && j == 5) || (i == 4 && j == 6))
                M[i][j] = -0.33;
            else if ((i == 4 && j == 5) || (i == 5 && j == 6))
                M[i][j] = -0.67;
            else
                M[i][j] = 0;
        }
    }
    for (int i=0; i<num; i++)
    {
        double* result = Simulation(alpha, beta, kappa, i==num-1);
        double costArray[8], waitArray[8], distArray[8];
        for (int k=0; k<8; k++)
        {
            costArray[k] = result[k];
            waitArray[k] = result[k+8];
            distArray[k] = result[k+16];
        }
        // Order the CNCs by different parameters
        int* costOrder = topBottomIndices(costArray);
        int* waitOrder = topBottomIndices(waitArray);
        int* distOrder = topBottomIndices(distArray);
        // The learning rate decreases over time
        double learnRate = RATE * power(0.3, i / 50);
        // Update the coefficients from results of the previous simulation
        int Acounter = -1;
        int Bcounter = -1;
        char* knives = knifeDistribution(knifeNum());
        for (int j=0; j<8; j++)
        {
            double rate;
            if (knives[costOrder[j]] == 'A')
            {
                Acounter++;
                rate = M[Acounter][knifeNum()-1] * learnRate;
                if (Acounter <= knifeNum() / 2 - 1)
                {
                    alpha[costOrder[j]] += rate;
                    beta[costOrder[j]] *= (1 - 2 * rate);
                    kappa[costOrder[j]] += rate;
                }
                else
                {
                    alpha[costOrder[j]] *= (1 + 2 * rate);
                    beta[costOrder[j]] -= rate;
                    kappa[costOrder[j]] *= (1 + 2 * rate);
                }
            }
            else
            {
                Bcounter++;
                rate = M[Bcounter][7-knifeNum()] * learnRate;
                if (Bcounter <= (8 - knifeNum()) / 2 - 1)
                {
                    alpha[costOrder[j]] += rate;
                    beta[costOrder[j]] *= (1 - 2 * rate);
                    kappa[costOrder[j]] += rate;
                }
                else
                {
                    alpha[costOrder[j]] *= (1 + 2 * rate);
                    beta[costOrder[j]] -= rate;
                    kappa[costOrder[j]] *= (1 + 2 * rate);
                }
            }
        }
        Acounter = -1;
        Bcounter = -1;
        for (int j=0; j<8; j++)
        {
            double rate;
            if (knives[waitOrder[j]] == 'A')
            {
                Acounter++;
                rate = M[Acounter][knifeNum()-1] * learnRate;
                if (Acounter <= knifeNum() / 2 - 1)
                    alpha[waitOrder[j]] += rate;
                else
                    alpha[waitOrder[j]] *= (1 + 2 * rate);
            }
            else
            {
                Bcounter++;
                rate = M[Bcounter][7-knifeNum()] * learnRate;
                if (Bcounter <= (8 - knifeNum()) / 2 - 1)
                    alpha[waitOrder[j]] += rate;
                else
                    alpha[waitOrder[j]] *= (1 + 2 * rate);
            }
        }
        Acounter = -1;
        Bcounter = -1;
        for (int j=0; j<8; j++)
        {
            double rate;
            if (knives[distOrder[j]] == 'A')
            {
                Acounter++;
                rate = M[Acounter][knifeNum()-1] * learnRate;
                if (Acounter <= knifeNum() / 2 - 1)
                    beta[distOrder[j]] *= (1 - 2 * rate);
                else
                    beta[distOrder[j]] -= rate;
            }
            else
            {
                Bcounter++;
                rate = M[Bcounter][7-knifeNum()] * learnRate;
                if (Bcounter <= (8 - knifeNum()) / 2 - 1)
                    beta[distOrder[j]] *= (1 - 2 * rate);
                else
                    beta[distOrder[j]] -= rate;
            }
        }
    }
    // Return the final coefficients
    static double coeff[24];
    for (int i=0; i<8; i++)
    {
        coeff[i] = alpha[i];
        coeff[i+8] = beta[i];
        coeff[i+16] = kappa[i];
    }
    return coeff;
}

int maxIndex(double a[8])
{
    int ans = 0;
    for (int i=1; i<8; i++)
    {
        if (a[i] > a[ans])
            ans = i;
    }
    return ans;
}

int minIndex(double a[8])
{
    int ans = 0;
    for (int i=1; i<8; i++)
    {
        if (a[i] < a[ans])
            ans = i;
    }
    return ans;
}

int* topBottomIndices(double a[8])
{
    static int indices[8];
    for (int i=0; i<8; i++)
        indices[i] = i;
    for (int i=0; i<8; i++)
    {
        for (int j=0; j<8; j++)
        {
            if (a[i] < a[j])
            {
                int temp = indices[i];
                indices[i] = indices[j];
                indices[j] = temp;
            }
        }
    }
    return indices;
}

double power(double base, int exponent)
{
    double ans = 1;
    for (int i=1; i<=exponent; i++)
        ans = ans * base;
    return ans;
}

int computeProb(int num)
{
    if (num <= 150)
        return 10000;
    else if (num <= 250)
        return 20000;
    else if (num <= 350)
        return 25000;
    else if (num <= 500)
        return 35000;
    else
        return 50000;
}
