#ifndef COMM_CODES
#define COMM_CODES

// 26 commands x 2 (request and response) = 52 commands
// 6 bits for code = 64 codes
// 6th bit is 0 for request and 1 for response
// 7 bits, one more bit to distinguish internal commands from user commands
// 7th bit is 0 for internal commands and 1 for user commands

constexpr byte request_code  = 0b1000000;
constexpr byte response_code = 0b1100000;
constexpr byte gli = 0;
constexpr byte gdi = 1;
constexpr byte goi = 2;
constexpr byte oiv = 3;
constexpr byte gOi = 4;
constexpr byte Oiv = 5;
constexpr byte gUi = 6;
constexpr byte Uiv = 7;
constexpr byte gLi = 8;
constexpr byte gxi = 9;
constexpr byte gri = 10;
constexpr byte gci = 11;
constexpr byte civ = 12;
constexpr byte gpi = 13;
constexpr byte gpT = 14;
constexpr byte gti = 15;
constexpr byte gei = 16;
constexpr byte geT = 17;
constexpr byte gvi = 18;
constexpr byte gvT = 19;
constexpr byte gfi = 20;
constexpr byte gfT = 21;
constexpr byte r = 22;
constexpr byte bxi = 23;
constexpr byte sxi1 = 24;
constexpr byte sxi2 = 25;

#endif //COM_CODES
