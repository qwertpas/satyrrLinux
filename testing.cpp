#include <sys/time.h>
#include <iostream>
#include <stdio.h>
#include <thread>
#include <chrono>


using namespace std;



struct timeval get_millis(){
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return tv;
}

int main(){
    long int sec = get_millis().tv_sec * 1e6;
    long int usec = get_millis().tv_usec;
    long int usec_total = sec + usec;
    float sec_total = usec_total * 1e-6;
    cout << sec_total << endl;



    this_thread::sleep_for(chrono::milliseconds(2000));

    long int sec2 = get_millis().tv_sec * 1e6;
    long int usec2 = get_millis().tv_usec;
    long int usec_total2 = sec2 + usec2;
    float sec_total2 = usec_total2 * 1e-6;
    cout << sec_total2 << endl;
}