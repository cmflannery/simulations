#include <iostream>

class Rocket {
    public:
        double thrust, g0;
    
    void state(double t, double g) {
        thrust  = t;
        g0      = g;
    };

    
};

int main() 
{
    Rocket launchy;
    launchy.state(4450,9.81);
    std::cout << launchy.thrust << " N" << std::endl;
    std::cout << launchy.g0 << " m/s^2" << std::endl;
    return 0;
};

