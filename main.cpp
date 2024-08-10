#include <iostream>
#include <boost/regex.hpp>

int main() {
    std::string s = "Boost Libraries";
    boost::regex expr{"(\\w+)\\s(\\w+)"};
    boost::smatch matches;

    if (boost::regex_search(s, matches, expr)) {
        std::cout << "Match found: " << matches[0] << std::endl;
        std::cout << "First word: " << matches[1] << std::endl;
        std::cout << "Second word: " << matches[2] << std::endl;
    } else {
        std::cout << "No match found" << std::endl;
    }

    return 0;
}
