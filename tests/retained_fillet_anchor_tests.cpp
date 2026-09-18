#include "SavedBooleanFilletProbe.hxx"
#include <iostream>
int main(){bool passed=true;
    for(unsigned scenario=0;scenario<4;++scenario){const auto checks=core3d::saved_boolean_fillet_probe::Run(scenario);
        if(checks.empty())passed=false;
        for(const auto& row:checks){std::cout<<scenario<<" "<<row.first<<" "<<(row.second?"PASS":"FAIL")<<"\n";passed=passed&&row.second;}}
    return passed?0:1;
}
