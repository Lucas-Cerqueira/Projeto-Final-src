// NS_LOG="YansWifiChannel" ./waf --run "channels-simulation --lossModel=6" 2>&1 | awk '/propagation/{print > "kunisch_urban.log"}'
// ./run "channels-simulation --lossModel=7" -f "propagation" -l "YansWifiChannel" -o "kunisch_rural.log"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>

#include <unistd.h>

using namespace std;

string executeCommand (const char* cmd)
{
    array<char, 128> buffer;
    string result;
    unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

vector<string> splitString (const string& s, char delimiter)
{
   vector<string> tokens;
   string token;
   istringstream tokenStream(s);
   while (getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

bool fileExists (const char* filename)
{
  ifstream ifile(filename);
  return (bool)ifile;
}

int main (int argc, char *argv[])
{
  string scriptFile;
  string scriptName;
  string options;

  string logModule;
  string outputFile;
  string filterString = ".*";

  int c;

  // Check for missing argument
  if (argc == 1)
  {
    cout << "ERROR: Missing script name" << endl;
    cout << "Examples of usage:" << endl <<
    "-> ./run scriptName" << endl <<
    "-> ./run \"scriptName --option1=value1 --option2=value2 ...\"" << endl;
    return -1;
  }

  // Get the script name from the argv string
  vector<string> results = splitString (argv[1], ' ');
  scriptFile = results[0];
  options = string(argv[1]).substr(scriptFile.length());

  // Parse the command-line arguments
  while ((c = getopt (argc, argv, "l:o:f:")) != -1)
  {
    switch (c)
    {
      case 'l': // Log module
        logModule = optarg;
        break;

      case 'o': // Output file
        outputFile = optarg;
        break;

      case 'f': // Filter string
        filterString = optarg;
        break;
    }
  }

  // Add ".cc" if the `scriptName` does not have an extension
  if (scriptFile.find('.') == string::npos)
  {
    scriptName = scriptFile;
    scriptFile += ".cc";
  }
  // If it does have an extension, then remove the extension and set `scriptName`
  else
  {
    size_t lastindex = scriptFile.find_last_of(".");
    scriptName = scriptFile.substr(0, lastindex);
  }

  // Check if the script exists
  scriptFile = "scripts/" + scriptFile;
  if (!fileExists(scriptFile.c_str()))
  {
    cout << "ERROR: File " << scriptFile << " not found." << endl;
    return -1;
  }

  // Copy script to ns3 scratch folder to be executed
  string command = "cp " + scriptFile + " ns-3-dev/scratch/.";
  executeCommand (command.c_str());

  // Configure the command based on the parsed arguments
  command = "cd ns-3-dev && ";
  if (!logModule.empty())
    command += "NS_LOG=\"" + logModule + "\" ";

  command += "./waf --run \"" + scriptName + options + "\"";

  if (!outputFile.empty())
    command += " 2>&1 | awk '/" + filterString + "/{print > \"" + outputFile + "\"}'";

  command += "; cd ..";

  // Run the command
  string result = executeCommand (command.c_str());
  cout << result << endl;

  // Copy the output file to the output folder
  if (!outputFile.empty())
  {
    command = "mv ns-3-dev/" + outputFile + " output/.";
    executeCommand (command.c_str());
  }
}
