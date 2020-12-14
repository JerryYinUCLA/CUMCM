#include <iostream>
#include <cmath>
#include <algorithm>

using namespace std;

const int DISTANCE[4] = {0, 22, 30, 46};
const int SERVINGTIME[2] = {28, 31};
const int WORK = 560;
const int WASH = 25;
const double RATE = 0.005;

double* simulation(double alpha[8], double beta[8], double kappa[8], bool last);
double* manySimulations(int num);
int maxIndex(double a[8]);
int minIndex(double a[8]);
int* topBottomIndices(double a[8]);
double power(double base, int exponent);

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

// One round of simulation
double* simulation(double alpha[8], double beta[8], double kappa[8], bool last)
{
    int waitTime[8], currentDist[8], cumuWait[8], cumuDist[8], workTime[8], counter[8];
    double cumuCost[8];
    // done specifies whether a CNC is waiting; firstTime specifies whether this is the CNC's first round of work, which determines whether we need to wash
    bool done[8], firstTime[8];
    int trackCNC[500], trackTime[500];
    for (int i=0; i<8; i++)
    {
        waitTime[i] = 0;
        currentDist[i] = DISTANCE[i / 2];
        cumuWait[i] = 0;
        cumuDist[i] = 0;
        workTime[i] = WORK;
        counter[i] = 0;
        cumuCost[i] = 0;
        done[i] = true;
        firstTime[i] = true;
    }
    double cost[8];
    // The initial location of the RGV
    int locRGV = 0;
    int num = 0;
    bool RGVFree;
    for (int time = 0; time<28800;)
    {
        num++;
        for (int i=0; i<8; i++)
        {
            // If a CNC is waiting, compute its cost of waiting; also accumulate this cost to calculate an average cost at the end
            if (done[i])
            {
                cost[i] = alpha[i] * waitTime[i] - beta[i] * currentDist[i] + kappa[i];
                counter[i]++;
                cumuCost[i] += cost[i];
            }
            // Otherwise, set its cost to a very small value so this CNC won't be chosen; note that here we do not accumulate this cost
            else
                cost[i] = -100000;
        }
        if (cost[maxIndex(cost)] > -100000)
        {
            RGVFree = false;
            // Choose a CNC to serve
            int nextToServe = maxIndex(cost);
            trackCNC[num-1] = nextToServe;
            // The number of steps needed to take
            int move = abs(nextToServe / 2 - locRGV / 2);
            trackTime[num-1] = time + 1 + DISTANCE[move];
            // Total time the RGV needs to serve the chosen CNC, including moving, uploading and maybe washing
            int duration;
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
            // For considered CNC, accumulate its distance from the RGV
            for (int i=0; i<8; i++)
                cumuDist[i] += currentDist[i];
            // Update the time variables: working and waiting time
            for (int i=0; i<8; i++)
            {
                if (i == nextToServe)
                {
                    waitTime[i] = 0;
                    workTime[i] = WORK;
                    done[i] = false;
                }
                else
                {
                    if (done[i])
                        waitTime[i] += duration;
                    else
                    {
                        workTime[i] -= duration;
                        if (workTime[i] <= 0)
                        {
                            waitTime[i] = abs(workTime[i]);
                            workTime[i] = 0;
                            done[i] = true;
                        }
                    }
                }
            }
            // Update each CNC's distance from the RGV
            for (int i=0; i<8; i++)
                currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
            // Update the position of the RGV
            locRGV = nextToServe;
            // Update the time counter
            time += duration;
        }
        else
        {
            RGVFree = true;
            for (int i=0; i<8; i++)
                cost[i] = max(DISTANCE[abs(locRGV / 2 - i / 2)], workTime[i]);
            int nextToServe = minIndex(cost);
            int duration = cost[minIndex(cost)] + SERVINGTIME[nextToServe % 2];
            trackCNC[num-1] = nextToServe;
            trackTime[num-1] = time + 1 + cost[minIndex(cost)];
            // Update the time variables: working and waiting time
            cumuWait[nextToServe] += max(0, workTime[nextToServe] - DISTANCE[abs(locRGV / 2 - nextToServe / 2)]);
            for (int i=0; i<8; i++)
            {
                if (i == nextToServe)
                {
                    waitTime[i] = 0;
                    workTime[i] = WORK;
                    done[i] = false;
                }
                else
                {
                    if (done[i])
                        waitTime[i] += duration;
                    else
                    {
                        workTime[i] -= duration;
                        if (workTime[i] <= 0)
                        {
                            waitTime[i] = abs(workTime[i]);
                            workTime[i] = 0;
                            done[i] = true;
                        }
                    }
                }
            }
            // Update each CNC's distance from the RGV
            for (int i=0; i<8; i++)
                currentDist[i] = DISTANCE[abs(nextToServe / 2 - i / 2)];
            // Update the position of the RGV
            locRGV = nextToServe;
            // Update the time counter
            time += duration;
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
        cout << num << endl;
        for (int i=0; i<num; i++)
            cout << i+1 << " " << trackCNC[i]+1 << " " << trackTime[i] << endl;
    }
    return results;
}

double* manySimulations(int num)
{
    double alpha[8], beta[8], kappa[8];
    // Initialize the coefficients randomly
    for (int i=0; i<8; i++)
    {
        alpha[i] = (rand() % 1009 + 1) / double(1010);
        beta[i] = (rand() % 1009 + 1) / double(1010);
        kappa[i] = 0;
    }
    for (int i=0; i<num; i++)
    {
        double* result = simulation(alpha, beta, kappa, i==num-1);
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
        for (int j=0; j<8; j++)
        {
            if (j <= 3)
            {
                double rate = 0.25 * (4 - j) * learnRate;
                alpha[costOrder[j]] += rate;
                beta[costOrder[j]] *= (1 - 2 * rate);
                kappa[costOrder[j]] += rate;
                alpha[waitOrder[j]] += rate;
                beta[distOrder[j]] *= (1 - 2 * rate);
            }
            else
            {
                double rate = 0.25 * (j - 3) * learnRate;
                alpha[costOrder[j]] *= (1 - 2 * rate);
                beta[costOrder[j]] += rate;
                kappa[costOrder[j]] *= (1 - 2 * rate);
                alpha[waitOrder[j]] *= (1 - 2 * rate);
                beta[distOrder[j]] += rate;
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
