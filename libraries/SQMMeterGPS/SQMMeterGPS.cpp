#include "Arduino.h"
#include "Time.h"

time_t tmConvert_t(int YYYY, byte MM, byte DD, byte hh, byte mm, byte ss) {
    tmElements_t tmSet;
    tmSet.Year = YYYY - 1970;
    tmSet.Month = MM;
    tmSet.Day = DD;
    tmSet.Hour = hh;
    tmSet.Minute = mm;
    tmSet.Second = ss;
    return makeTime(tmSet);         //convert to time_t
}

int getBortle(double fst) {
    int bortle;
  
    if (fst >= 7.6 && fst <= 8.0) {
        bortle = 1;
    } else if (fst >= 7.00 && fst <= 7.50) {
        bortle = 2;
    } else if (fst >= 6.50 && fst < 7.00) {
        bortle = 3;
    } else if (fst >= 6.00 && fst < 6.50) {
        bortle = 4;
    } else if (fst >= 5.50 && fst < 6.00) {
        bortle = 5;
    } else if (fst >= 5.00 && fst < 5.50) {
        bortle = 6;
    } else if (fst >= 4.50 && fst < 5.00) {
        bortle = 7;
    } else if (fst >= 4.00 && fst < 4.50) {
        bortle = 8;
    } else if (fst <= 4.00) {
        bortle = 9;
    } else {
        bortle = 99;
    }
    return bortle;
}