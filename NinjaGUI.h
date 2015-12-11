#ifndef NINJAGUI_H
#define NINJAGUI_H

#endif // NINJAGUI_H

#include <iostream>
#include <functional>
#include <pangolin/pangolin.h>
#include <thread>


class NinjaGUI
{
private:
    std::thread     GUIThread;
    void GUImain(void);
    void GlobalKeyHook(const std::string& example);

public:
    NinjaGUI();
    ~NinjaGUI();


};
