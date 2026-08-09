#include "post_runner.hpp"
#include <iostream>

int main(int argc, char** argv)
{
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <studentId> <resultsDir>\n";
    return 1;
  }
  using namespace poster;
  FileResultSource source(argv[2]);
  HttpResultSink sink("http://cppmiltech.com.ua", "dz12-vX7mK4qT9r2w");
  RealSleeper sleeper;
  PostRunner runner(source, sink, argv[1], sleeper);
  try {
    auto reports = runner.runAll({"T01", "T02", "T03", "T04", "T05", "T06", "T07", "T08", "T09", "T10"});
    printReport(reports, std::cout);
    return 0;
  }
  catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }
}