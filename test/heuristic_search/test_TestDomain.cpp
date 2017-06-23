/* heuristic_search library
 *
 * Copyright (c) 2016, 
 * Maciej Przybylski <maciej.przybylski@mchtr.pw.edu.pl>, 
 * Warsaw University of Technology.
 * All rights reserved.
 *  
 */

#include "../heuristic_search/test_TestDomain.h"

#include <iostream>

namespace test_heuristic_search{

}

std::ostream& operator<<(std::ostream& stream, test_heuristic_search::TestDomain_::State const& state)
{
    stream << "state.id: " << state.id << "\t";    
    return stream;
}
