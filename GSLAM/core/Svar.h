// GSLAM - A general SLAM framework and benchmark
// Copyright 2018 PILAB Inc. All rights reserved.
// https://github.com/zdzhaoyong/GSLAM
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: zd5945@126.com (Yong Zhao) 353184965@qq.com(Guochen Liu)
//
// Svar: A light-weight, efficient, thread-safe parameter setting, dynamic
// variable sharing and command calling util class.
// Features:
// * Arguments parsing with help information
// * Support a very tiny script language with variable, function and condition
// * Thread-safe variable binding and sharing
// * Function binding and calling with Scommand
// * Support tree structure presentation, save&load with XML, JSON and YAML formats

#ifndef GSLAM_SVAR_H
#define GSLAM_SVAR_H
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <typeinfo>
#include <vector>

#define svar GSLAM::Svar::instance()
#define scommand GSLAM::Scommand::instance()
#define GSLAM_API
#define CallBack(FUNCNAME)                                          \
  static void FUNCNAME##CB(void* t, std::string a, std::string b) { \
    C(t).FUNCNAME(a, b);                                            \
  }                                                                 \
  void FUNCNAME(std::string name, std::string args)

namespace GSLAM {

class Svar;
class Scommand;
class SvarLanguage;
/** The class Svar will be shared in the same process, it help users to
 transform paraments use a name id,
 all paraments with a same name will got the same data. One can change it in all
 threads from assigned var,
 files and stream.
 */

template <typename VarType = void*, typename KeyType = std::string>
class SvarWithType {
  friend class Svar;

 public:
  typedef std::map<KeyType, VarType> DataMap;
  typedef typename DataMap::iterator DataIter;
  typedef std::pair<DataIter, bool> InsertRet;

 public:
  SvarWithType() {}

  /** This gives us singletons instance. \sa enter */
  static SvarWithType& instance() {
    static std::shared_ptr<SvarWithType> inst(new SvarWithType());
    return *inst;
  }

  inline bool exist(const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    return data.find(name) != data.end();
  }

  inline bool erase(const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    data.erase(name);
    return true;
  }

  inline bool clear() {
    std::unique_lock<std::mutex> lock(mMutex);
    data.clear();
    return true;
  }

  /** This insert a named var to the map,you can overwrite or not if the var has
 * exist. \sa enter
*/
  inline bool insert(const KeyType& name, const VarType& var,
                     bool overwrite = false) {
    std::unique_lock<std::mutex> lock(mMutex);
    DataIter it;
    it = data.find(name);
    if (it == data.end()) {
      data.insert(std::pair<std::string, VarType>(name, var));
      return true;
    } else {
      if (overwrite) it->second = var;
      return false;
    }
  }

  /** function get_ptr() returns the pointer of the map or the var pointer when
 * name is supplied,
 * when the var didn't exist,it will return NULL or insert the default var and
 * return the var pointer in the map
 */
  inline DataMap* get_ptr() { return &data; }

  inline const DataMap& get_data() { return data; }

  inline VarType* get_ptr(const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    DataIter it;
    it = data.find(name);
    if (it == data.end()) {
      return NULL;
    } else {
      return &(it->second);
    }
  }

  inline VarType* get_ptr(const KeyType& name, const VarType& def) {
    std::unique_lock<std::mutex> lock(mMutex);
    DataIter it;
    it = data.find(name);
    if (it == data.end()) {
      InsertRet ret = data.insert(std::pair<KeyType, VarType>(name, def));
      if (ret.second)
        return &(ret.first->second);
      else
        return NULL;
    } else {
      return &(it->second);
    }
  }

  /** function get_var() return the value found in the map,\sa enter.
 */
  inline VarType get_var(const KeyType& name, const VarType& def) {
    std::unique_lock<std::mutex> lock(mMutex);
    DataIter it;
    it = data.find(name);
    if (it == data.end()) {
      InsertRet ret = data.insert(std::pair<KeyType, VarType>(name, def));
      if (ret.second)
        return (ret.first->second);
      else
        return def;
    } else {
      return it->second;
    }
  }

  /** this function can be used to assign or get the var use corrospond name,\sa
 * enter.
 */
  inline VarType& operator[](const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    DataIter it;
    it = data.find(name);
    if (it == data.end()) {
      while (1) {
        InsertRet ret =
            data.insert(std::pair<KeyType, VarType>(name, VarType()));
        if (ret.second) return (ret.first->second);
      }
      //            else return def;//UNSAFE!!!!!
    } else {
      return it->second;
    }
  }

  std::string getStatsAsText() {
    std::unique_lock<std::mutex> lock(mMutex);
    std::ostringstream ost;

    for (DataIter it = data.begin(); it != data.end(); it++)
      ost << std::setw(39) << std::setiosflags(std::ios::left) << it->first
          << "  " << std::setw(39) << std::setiosflags(std::ios::left)
          << it->second << std::endl;
    std::string a;
    return ost.str();
  }

  /** this print all the names with var to the screen,\sa enter.
 */
  void dumpAllVars(const size_t column_width = 80) {
    std::cout << std::endl << getStatsAsText(column_width);
  }

 protected:
  std::map<KeyType, VarType> data;

  std::mutex mMutex;
};  // end of class SvarWithType

typedef SvarWithType<int> SInt;
typedef SvarWithType<double> SDouble;
typedef SvarWithType<std::string> SString;

class GSLAM_API Svar {
 public:
  typedef std::map<std::string, std::string> SvarMap;
  typedef std::map<std::string, std::string>::iterator SvarIter;

  enum SVARMODE {
    SILENT = 0x00,
    UPDATE = 0x01,
    VERBOSE = 0x02,
    OVERWRITE = 0x04
  };

  enum PARSEMODE { DEFAULT_CMD1 = 0, DEFAULT_CMD2 = 1, CMD1 = 2, CMD_2 = 3 };

  struct ArgumentInfo {
    std::string type, def, introduction;
  };

 public:
  Svar();
  ~Svar();

  /** This gives us singletons instance. \sa enter */
  static Svar& instance();

  /** \brief update svar
 */
  bool insert(std::string name, std::string var, bool overwrite = true);
  std::string getvar(std::string name);

  bool ParseLine(std::string s, bool bSilentFailure = false);
  bool ParseStream(std::istream& is);
  bool ParseFile(std::string sFileName);

  template <typename T>
  void Arg(const std::string& name, T def, const std::string& info);
  std::vector<std::string> ParseMain(int argc, char** argv,
                                     PARSEMODE mode = DEFAULT_CMD1);
  std::string help();

  /** \brief
 */
  bool exist(const std::string& name);

  template <class T>
  T& Get(const std::string& name, T def = T());

  int& GetInt(const std::string& name, int defaut = 0, SVARMODE mode = SILENT);
  double& GetDouble(const std::string& name, double defaut = 0,
                    SVARMODE mode = SILENT);
  std::string& GetString(const std::string& name,
                         const std::string& defaut = "",
                         SVARMODE mode = SILENT);
  void*& GetPointer(const std::string& name, const void* p = NULL,
                    SVARMODE mode = SILENT);
  Svar GetChild(const std::string& name);

  std::shared_ptr<SvarWithType<Svar> >& Children() { return a->c; }
  Scommand& command();

  bool erase(const std::string& name);
  void update();
  const SvarMap& get_data();

  /** \brief clear Svar data
*/
  void clear();

  /** \brief clear all data including SvarWithType<int>, SvarWithType<double>,
 * SvarWithType<string>
*/
  void clearAll();

  /** \brief other utils
 */
  std::string getStatsAsText();
  void dumpAllVars();

  bool save2file(std::string filename = "");

  static std::string dtos(const double& i, int precision);
  static std::string getFolderPath(const std::string& path);
  static std::string getBaseName(const std::string& path);
  static std::string getFileName(const std::string& path);
  static std::vector<std::string> ChopAndUnquoteString(std::string s);

  template <class T>
  T get_var(const std::string& name, const T& def);  // deprecated since 2.4.5
 private:
  bool setvar(std::string s);  // eg. setvar("var=val");
  std::string expandVal(std::string val, char flag = '{');

  static std::string UncommentString(std::string s);
  static const char* FirstOpenBrace(const char* str, char flag = '{');
  static const char* MatchingEndBrace(const char* str, char flag = '{');
  static std::string Trim(const std::string& str,
                          const std::string& delimiters = " \f\n\r\t\v");
  static std::string::size_type FindCloseBrace(const std::string& s,
                                               std::string::size_type start,
                                               char op, char cl);

  static std::string typeName(std::string name);

 protected:
  std::shared_ptr<SvarWithType<std::string> > data;
  struct Data {
    std::shared_ptr<SvarWithType<int> > i;
    std::shared_ptr<SvarWithType<double> > d;
    std::shared_ptr<SvarWithType<std::string> > s;
    std::shared_ptr<SvarWithType<Svar> > c;
    std::shared_ptr<SvarWithType<void*> > p;
    std::shared_ptr<Scommand> parser;
    std::shared_ptr<SvarWithType<ArgumentInfo> > args;
  };
  std::shared_ptr<Data> a;
};  // end of class Svar

///
/// ptr      - class pointer
/// sCommand - command string
/// sParams  - parameters
///
typedef void (*CallbackProc)(void* ptr, std::string sCommand,
                             std::string sParams);

struct CallbackInfoStruct {
  CallbackInfoStruct(CallbackProc callback, void* ptr)
      : cbp(callback), thisptr(ptr) {}

  void Call(std::string sCommand = "", std::string sParams = "") {
    cbp(thisptr, sCommand, sParams);
  }

  CallbackProc cbp;
  void* thisptr;
};

typedef std::vector<CallbackInfoStruct> CallbackVector;

class GSLAM_API Scommand {
 public:
  Scommand(Svar& var = Svar::instance());
  static Scommand& instance();

  void RegisterCommand(std::string sCommandName, CallbackProc callback,
                       void* thisptr = NULL);
  void UnRegisterCommand(std::string sCommandName);
  void UnRegisterCommand(std::string sCommandName, void* thisptr);
  void UnRegisterCommand(void* thisptr);

  bool Call(std::string sCommand, std::string sParams);
  bool Call(const std::string& sCommand);

 protected:
  SvarWithType<CallbackVector> data;
  std::shared_ptr<SvarLanguage> language;
};

class SvarLanguage {
 public:
  SvarLanguage(Scommand& command = Scommand::instance(),
               Svar& var = Svar::instance())
      : scommand_(command), svar_(var) {
    scommand_.RegisterCommand(".", collect_lineCB, this);
    scommand_.RegisterCommand("function", functionCB, this);
    scommand_.RegisterCommand("endfunction", endfunctionCB, this);
    scommand_.RegisterCommand("if", gui_if_equalCB, this);
    scommand_.RegisterCommand("else", gui_if_elseCB, this);
    scommand_.RegisterCommand("endif", gui_endifCB, this);
  }

  ~SvarLanguage() {}

 private:
  std::string current_function;
  std::string if_gvar, if_string;
  std::vector<std::string> collection, ifbit, elsebit;
  SvarWithType<std::vector<std::string> > functions;

  static SvarLanguage& C(void* v) { return *static_cast<SvarLanguage*>(v); }

  CallBack(collect_line) {
    (void)name;
    collection.push_back(args);
  }

  CallBack(function) {
    using namespace std;

    svar_.GetInt("Svar.Collecting", 0)++;
    vector<string> vs = Svar::ChopAndUnquoteString(args);
    if (vs.size() != 1) {
      cerr << "Error: " << name << " takes 1 argument: " << name << " name\n";
      return;
    }

    current_function = vs[0];
    collection.clear();
  }

  CallBack(endfunction) {
    using namespace std;
    svar_.GetInt("Svar.Collecting", 0)--;
    if (current_function == "") {
      cerr << "Error: " << name << ": no current function.\n";
      return;
    }

    vector<string> vs = Svar::ChopAndUnquoteString(args);
    if (vs.size() != 0) cerr << "Warning: " << name << " takes 0 arguments.\n";

    functions.insert(current_function, collection, true);

    scommand_.RegisterCommand(current_function, runfunctionCB, this);

    current_function.clear();
    collection.clear();
  }

  CallBack(runfunction) {
    (void)args;
    using namespace std;
    vector<string>& v = *functions.get_ptr(name, vector<string>());
    for (unsigned int i = 0; i < v.size(); i++) svar_.ParseLine(v[i]);
  }

  CallBack(gui_if_equal) {
    (void)name;
    using namespace std;
    string& s = args;
    svar_.GetInt("Svar.Collecting", 0)++;
    bool is_equal = false;
    string::size_type n;
    n = s.find("=");
    if (n != string::npos) {
      string left = s.substr(0, n);
      string right = s.substr(n + 1);
      // Strip whitespace from around left;
      string::size_type s = 0, e = left.length() - 1;
      if ('!' == left[e]) {
        //                        cout<<"Found !"<<endl;
        e--;
        is_equal = true;
      }
      for (; isspace(left[s]) && s < left.length(); s++) {
      }
      if (s == left.length())  // All whitespace before the `='?
        left = "";
      else
        for (; isspace(left[e]); e--) {
        }
      if (e >= s) {
        left = left.substr(s, e - s + 1);
      } else
        left = "";

      // Strip whitespace from around val;
      s = 0, e = right.length() - 1;
      for (; isspace(right[s]) && s < right.length(); s++) {
      }
      if (s < right.length()) {
        for (; isspace(right[e]); e--) {
        }
        right = right.substr(s, e - s + 1);
      } else
        right = "";

      //                    cout<<"Found
      //                    =,Left:-"<<left<<"-,Right:-"<<right<<"-\n";

      if (left == right) is_equal = !is_equal;
    } else if (s != "") {
      is_equal = true;
    }

    collection.clear();
    if (is_equal)
      if_gvar = "";
    else
      if_gvar = "n";
    if_string = "";
  }

  CallBack(gui_if_else) {
    (void)name;
    (void)args;
    using namespace std;
    ifbit = collection;
    if (ifbit.empty()) ifbit.push_back("");
    collection.clear();
  }

  CallBack(gui_endif) {
    (void)name;
    (void)args;
    using namespace std;
    svar_.GetInt("Svar.Collecting", 0)--;
    if (ifbit.empty())
      ifbit = collection;
    else
      elsebit = collection;

    collection.clear();

    // Save a copy, since it canget trashed
    vector<string> ib = ifbit, eb = elsebit;
    string gv = if_gvar, st = if_string;

    ifbit.clear();
    elsebit.clear();
    if_gvar.clear();
    if_string.clear();
    //                cout<<"SvarName="<<gv<<",Value="<<svar.GetString(gv,"")<<",Test="<<st<<endl;
    if (gv == st)
      for (unsigned int i = 0; i < ib.size(); i++) svar_.ParseLine(ib[i]);
    else
      for (unsigned int i = 0; i < eb.size(); i++) svar_.ParseLine(eb[i]);
  }

  Scommand& scommand_;
  Svar svar_;
};

inline std::string Svar::Trim(const std::string& str,
                              const std::string& delimiters) {
  const size_t f = str.find_first_not_of(delimiters);
  return f == std::string::npos
             ? ""
             : str.substr(f, str.find_last_not_of(delimiters) + 1);
}

// Find the open brace preceeded by '$'
inline const char* Svar::FirstOpenBrace(const char* str, char flag) {
  bool symbol = false;

  for (; *str != '\0'; ++str) {
    if (*str == '$') {
      symbol = true;
    } else {
      if (symbol) {
        if (*str == flag) {
          return str;
        } else {
          symbol = false;
        }
      }
    }
  }
  return 0;
}

// Find the first matching end brace. str includes open brace
inline const char* Svar::MatchingEndBrace(const char* str, char flag) {
  char endflag = '}';
  if (flag == '(')
    endflag = ')';
  else if (flag == '[')
    endflag = ']';
  int b = 0;
  for (; *str != '\0'; ++str) {
    if (*str == flag) {
      ++b;
    } else if (*str == endflag) {
      --b;
      if (b == 0) {
        return str;
      }
    }
  }
  return 0;
}

inline std::vector<std::string> Svar::ChopAndUnquoteString(std::string s) {
  using namespace std;
  vector<string> v;
  string::size_type nPos = 0;
  string::size_type nLength = s.length();
  while (1) {
    string sTarget;
    char cDelim;
    // Get rid of leading whitespace:
    while ((nPos < nLength) && (s[nPos] == ' ')) nPos++;
    if (nPos == nLength) return v;

    // First non-whitespace char...
    if (s[nPos] != '\"')
      cDelim = ' ';
    else {
      cDelim = '\"';
      nPos++;
    }
    for (; nPos < nLength; ++nPos) {
      char c = s[nPos];
      if (c == cDelim) break;
      if (cDelim == '"' && nPos + 1 < nLength && c == '\\') {
        char escaped = s[++nPos];
        switch (escaped) {
          case 'n':
            c = '\n';
            break;
          case 'r':
            c = '\r';
            break;
          case 't':
            c = '\t';
            break;
          default:
            c = escaped;
            break;
        }
      }
      sTarget += c;
    }
    v.push_back(sTarget);

    if (cDelim == '\"') nPos++;
  }
}

inline std::string::size_type Svar::FindCloseBrace(const std::string& s,
                                                   std::string::size_type start,
                                                   char op, char cl) {
  using namespace std;
  string::size_type open = 1;
  string::size_type i;

  for (i = start + 1; i < s.size(); i++) {
    if (s[i] == op)
      open++;
    else if (s[i] == cl)
      open--;

    if (open == 0) break;
  }

  if (i == s.size()) i = s.npos;
  return i;
}

inline std::string Svar::UncommentString(std::string s) {
  using namespace std;
  // int n = s.find("//");
  // return s.substr(0,n);

  int q = 0;

  for (string::size_type n = 0; n < s.size(); n++) {
    if (s[n] == '"') q = !q;

    if (s[n] == '/' && !q) {
      if (n < s.size() - 1 && s[n + 1] == '/') return s.substr(0, n);
    }
  }

  return s;
}
inline std::string Svar::dtos(const double& i, int precision) {
  using namespace std;
  ostringstream ost;
  ost << setiosflags(ios::fixed) << setprecision(precision) << i;
  return ost.str();
}

inline std::string Svar::getFolderPath(const std::string& path) {
  auto idx = std::string::npos;
  if ((idx = path.find_last_of('/')) == std::string::npos)
    idx = path.find_last_of('\\');
  if (idx != std::string::npos)
    return path.substr(0, idx);
  else
    return "";
}

inline std::string Svar::getBaseName(const std::string& path) {
  std::string filename = getFileName(path);
  auto idx = filename.find_last_of('.');
  if (idx == std::string::npos)
    return filename;
  else
    return filename.substr(0, idx);
}

inline std::string Svar::getFileName(const std::string& path) {
  auto idx = std::string::npos;
  if ((idx = path.find_last_of('/')) == std::string::npos)
    idx = path.find_last_of('\\');
  if (idx != std::string::npos)
    return path.substr(idx + 1);
  else
    return path;
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline Svar::Svar() : data(new SvarWithType<std::string>()), a(new Data()) {}

inline Svar::~Svar() {
  if (GetInt("Svar.DumpAllVars")) dumpAllVars();
}

inline Svar& Svar::instance() {
  static std::shared_ptr<Svar> global_svar(new Svar);
  return *global_svar;
}

inline void Svar::clear() { data->clear(); }

inline void Svar::clearAll() { data->clear(); }

inline bool Svar::erase(const std::string& name) {
  data->erase(name);
  return true;
}

inline bool Svar::exist(const std::string& name) { return data->exist(name); }

inline const Svar::SvarMap& Svar::get_data() { return data->get_data(); }

inline bool Svar::insert(std::string name, std::string var,
                         bool overwrite)  // default overwrite
{
  data->insert(name, var, overwrite);
}

inline std::string Svar::getvar(std::string name) {
  if (data->exist(name)) return data->get_var(name, "");
  return "";
}

inline std::string Svar::expandVal(std::string val, char flag) {
  using namespace std;
  string expanded = val;

  while (true) {
    const char* brace = FirstOpenBrace(expanded.c_str(), flag);
    if (brace) {
      const char* endbrace = MatchingEndBrace(brace, flag);
      if (endbrace) {
        ostringstream oss;
        oss << std::string(expanded.c_str(), brace - 1);

        const string inexpand =
            expandVal(std::string(brace + 1, endbrace), flag);
        if (exist(inexpand)) {
          oss << getvar(inexpand);
        } else {
          printf(
              "Unabled to expand: [%s].\nMake sure it is defined and "
              "terminated with a semi-colon.\n",
              inexpand.c_str());
          oss << "#";
        }

        oss << std::string(endbrace + 1, expanded.c_str() + expanded.length());
        expanded = oss.str();
        continue;
      }
    }
    break;
  }

  return expanded;
}

/**
 = overwrite
?= don't overwrite
*/
inline bool Svar::setvar(std::string s) {
  using namespace std;
  // Execution failed. Maybe its an assignment.
  string::size_type n;
  n = s.find("=");
  bool shouldOverwrite = true;

  if (n != string::npos) {
    string var = s.substr(0, n);
    string val = s.substr(n + 1);

    // Strip whitespace from around var;
    string::size_type s = 0, e = var.length() - 1;
    if ('?' == var[e]) {
      e--;
      shouldOverwrite = false;
    }
    for (; isspace(var[s]) && s < var.length(); s++) {
    }
    if (s == var.length())  // All whitespace before the `='?
      return false;
    for (; isspace(var[e]); e--) {
    }
    if (e >= s) {
      var = var.substr(s, e - s + 1);

      // Strip whitespace from around val;
      s = 0, e = val.length() - 1;
      for (; isspace(val[s]) && s < val.length(); s++) {
      }
      if (s < val.length()) {
        for (; isspace(val[e]); e--) {
        }
        val = val.substr(s, e - s + 1);
      } else
        val = "";

      insert(var, val, shouldOverwrite);
      return true;
    }
  }

  return false;
}

template <class T>
T& Svar::Get(const std::string& name, T def) {
  // First: Use the var from SvarWithType, this is a fast operation
  SvarWithType<T>& typed_map = SvarWithType<T>::instance();

  T* ptr = typed_map.get_ptr(name);
  if (ptr) return *ptr;

  std::string envStr;
  if (data->exist(name))
    envStr = (*data)[name];
  else if (char* envcstr = getenv(name.c_str()))
    envStr = envcstr;
  if (!envStr.empty())  // Second: Use the var from Svar
  {
    std::istringstream istr_var(envStr);
    try {
      istr_var >> def;
    } catch (std::exception e) {
      std::cerr << "Failed to read value from " << envStr << std::endl;
    }
  }

  ptr = typed_map.get_ptr(name, def);
  return *ptr;
}

template <>
inline int& Svar::Get(const std::string& name, int def) {
  return GetInt(name, def);
}

template <>
inline double& Svar::Get(const std::string& name, double def) {
  return GetDouble(name, def);
}

template <>
inline std::string& Svar::Get(const std::string& name, std::string def) {
  return GetString(name, def);
}

template <>
inline void*& Svar::Get(const std::string& name, void* def) {
  return GetPointer(name, def);
}

template <class T>
T Svar::get_var(const std::string& name, const T& def) {
  return Get<T>(name, def);
}

inline Scommand& Svar::command() {
  if (!a->parser) a->parser = std::shared_ptr<Scommand>(new Scommand(*this));
  return *(a->parser);
}

inline bool Svar::ParseLine(std::string s, bool bSilentFailure) {
  using namespace std;
  if (s == "") return 0;
  int& collectFlag = GetInt("Svar.Collecting", 0);
  if (collectFlag) {
    istringstream ist(s);
    string sCommand;
    ist >> sCommand;
    if (sCommand == "endif" || sCommand == "fi") command().Call("endif");
    if (sCommand == "else")
      command().Call("else");
    else if (sCommand == "endfunction")
      command().Call("endfunction");
    else if (sCommand == ".") {
      command().Call(".", ist.str());
    } else
      command().Call(".", s);
    return 0;
  }
  s = UncommentString(expandVal(s, '{'));
  s = UncommentString(expandVal(s, '('));
  if (s == "") return 0;

  // Old ParseLine code follows, here no braces can be left (unless in arg.)
  istringstream ist(s);

  string sCommand;
  string sParams;

  // Get the first token (the command name)
  ist >> sCommand;
  if (sCommand == "") return 0;

  // Get everything else (the arguments)...

  // Remove any whitespace
  ist >> ws;
  getline(ist, sParams);

  // Attempt to execute command
  if (command().Call(sCommand, sParams)) return true;

  if (setvar(s)) return 1;

  if (!bSilentFailure)
    cerr << "? GUI_impl::ParseLine: Unknown command \"" << sCommand
         << "\" or invalid assignment." << endl;
  return 0;
}

inline bool Svar::ParseStream(std::istream& is) {
  using namespace std;
  string parsingFile = GetString("Svar.ParsingFile", "");
  //    cout<<"ParsingFile:
  //    "<<parsingFile<<"\nParsingPath:"<<filePath.getFolderName()<<endl;
  insert("Svar.ParsingPath", getFolderPath(parsingFile), true);
  insert("Svar.ParsingName", getBaseName(parsingFile), true);
  insert("Svar.ParsingFile", parsingFile, true);
  string buffer;
  int& shouldParse = GetInt("Svar.NoReturn", 1);
  while (getline(is, buffer) && shouldParse) {
    // Lines ending with '\' are taken as continuing on the next line.
    while (!buffer.empty() && buffer[buffer.length() - 1] == '\\') {
      string buffer2;
      if (!getline(is, buffer2)) break;
      buffer = buffer.substr(0, buffer.length() - 1) + buffer2;
    }
    ParseLine(buffer);
  }
  shouldParse = 1;
  return true;
}

inline bool Svar::ParseFile(std::string sFileName) {
  using namespace std;
  static deque<string> fileQueue;

  ifstream ifs(sFileName.c_str());

  if (!ifs.is_open()) {
    //    cerr << "!Svar::ParseFile: Failed to load script file \"" << sFileName
    //         << "\"." << endl;
    return false;
  }

  fileQueue.push_back(sFileName);
  GetString("Svar.ParsingFile", sFileName) = sFileName;

  bool ret = ParseStream(ifs);
  ifs.close();

  //    cout<<"Finished parsing "<<fileQueue.back();
  fileQueue.pop_back();
  if (fileQueue.size()) {
    //        cout<<"Back to parsing "<<fileQueue.back();
    GetString("Svar.ParsingFile", sFileName) = fileQueue.back();
    string parsingFile = fileQueue.back();
    insert("Svar.ParsingPath", getFolderPath(parsingFile), true);
    insert("Svar.ParsingName", getFileName(parsingFile), true);
    insert("Svar.ParsingFile", parsingFile, true);
  } else {
    erase("Svar.ParsingName");
    erase("Svar.ParsingPath");
    erase("Svar.ParsingFile");
  }
  return ret;
}

template <typename T>
void Svar::Arg(const std::string& name, T def, const std::string& help) {
  std::stringstream sst;
  sst << def;
  if (!sst.str().empty()) insert(name, sst.str(), false);
  if (!a->args)
    a->args = std::shared_ptr<SvarWithType<ArgumentInfo> >(
        new SvarWithType<ArgumentInfo>());
  ArgumentInfo& argInfo = (*a->args)[name];
  argInfo.introduction = help;
  argInfo.type = typeid(T).name();
  argInfo.def = sst.str();
}

inline std::vector<std::string> Svar::ParseMain(int argc, char** argv,
                                                PARSEMODE mode) {
  using namespace std;
  // save main cmd things
  GetInt("argc") = argc;
  GetPointer("argv", NULL) = argv;
  // SvarWithType<char**>::instance()["argv"] = argv;

  // parse main cmd
  std::vector<std::string> unParsed;
  int beginIdx = (mode == DEFAULT_CMD1 ? 1 : 2);
  for (int i = beginIdx; i < argc; i++) {
    string str = argv[i];
    bool foundPrefix = false;
    int j = 0;
    for (; j < 2 && j < str.size() && str.at(j) == '-'; j++) foundPrefix = true;

    if (!foundPrefix) {
      if (!setvar(str)) unParsed.push_back(str);
      continue;
    }

    str = str.substr(j);
    if (str.find('=') != string::npos) {
      setvar(str);
      continue;
    }

    if (i + 1 >= argc) {
      insert(str, "1", true);
      continue;
    }
    string str2 = argv[i + 1];
    if (str2.front() == '-') {
      insert(str, "1", true);
      continue;
    }

    i++;
    insert(str, argv[i]);
    continue;
  }

  // parse default config file
  string cfg_File = argv[0];
  insert("argv0", cfg_File);
  insert("ProgramPath", getFolderPath(cfg_File));
  insert("ProgramName", getFileName(cfg_File));
  cfg_File += ".cfg";
  ifstream ifs(cfg_File.c_str());

  if (!ifs.is_open()) {
    cfg_File = "./Default.cfg";
  } else
    ifs.close();

  cfg_File = GetString("conf", cfg_File);
  // cout << "Parsing file: " << cfg_File << " ...." << endl;
  bool ret = ParseFile(cfg_File);

  // parse input argument again,FIXME:WHY
  //  for (int i = 1; i < argc; i++) setvar(argv[i]);
  if (exist("help")) std::cerr << help() << std::endl;
  return unParsed;
}

inline std::string Svar::typeName(std::string name) {
  static std::map<std::string, std::string> decode = {
      {typeid(int32_t).name(), "int32_t"},
      {typeid(int64_t).name(), "int64_t"},
      {typeid(uint32_t).name(), "uint32_t"},
      {typeid(uint64_t).name(), "uint64_t"},
      {typeid(u_char).name(), "u_char"},
      {typeid(char).name(), "char"},
      {typeid(float).name(), "float"},
      {typeid(double).name(), "double"},
      {typeid(std::string).name(), "string"},
      {typeid(bool).name(), "bool"}};
  auto it = decode.find(name);
  if (it == decode.end())
    return name;
  else
    return it->second;
}

inline std::string Svar::help() {
  if (Get<int>("help") == 0) return GetString("help");
  std::stringstream sst;
  sst << "Usage:\n"
      << GetString("ProgramName", "exe") << " [--help] [-conf configure_file]"
                                            " [-arg_name arg_value]...\n\n";

  if (!GetString("Version").empty())
    sst << "Version: " << GetString("Version") << ", ";
  sst << "Using Svar supported argument parsing. The following table listed "
         "several \nargument introductions.\n\n";

  Arg<std::string>("conf", "Default.cfg",
                   "The default configure file going to parse.");
  Arg<bool>("help", false, "Show the help information.");
  int namePartWidth = GetInt("NamePartWidth", 16);
  int statusPartWidth = GetInt("StatusPartWidth", 30);
  int introPartWidth = GetInt("IntroPartWidth", 40);
  auto& inst = *(a->args);
  sst << std::setw(namePartWidth + 1) << std::setiosflags(std::ios::left)
      << "argument" << std::setw(statusPartWidth + 1) << "type(default->setted)"
      << std::setiosflags(std::ios::left) << "introduction" << std::endl;
  sst << "-------------------------------------------------------------------"
         "-----------\n";
  for (const auto& it : inst.get_data()) {
    ArgumentInfo info = it.second;
    std::string setted = getvar(it.first);
    if (setted != info.def)
      setted = "->" + setted;
    else
      setted.clear();
    std::string name = "-" + it.first;
    std::string status = typeName(info.type) + "(" + info.def + setted + ")";
    std::string intro = info.introduction;

    for (; name.size() || status.size() || intro.size();) {
      sst << std::setw(namePartWidth) << name.substr(0, namePartWidth) << " "
          << std::setw(statusPartWidth) << status.substr(0, statusPartWidth)
          << " " << std::setw(introPartWidth) << intro.substr(0, introPartWidth)
          << std::endl;
      name = namePartWidth < name.size() ? name.substr(namePartWidth) : "";
      status =
          statusPartWidth < status.size() ? status.substr(statusPartWidth) : "";
      intro = introPartWidth < intro.size() ? intro.substr(introPartWidth) : "";
    }
  }
  return sst.str();
}

inline int& Svar::GetInt(const std::string& name, int def, SVARMODE mode) {
  using namespace std;
  if (!a->i)
    a->i = std::shared_ptr<SvarWithType<int> >(new SvarWithType<int>());
  // First: Use the var from SvarWithType, this is a fast operation
  SvarWithType<int>& typed_map = *(a->i);

  int* ptr = typed_map.get_ptr(name);
  if (ptr) return *ptr;

  bool exist = data->exist(name);
  char* envStr = NULL;
  if (!exist) envStr = getenv(name.c_str());
  if (exist || envStr)  // Second: Use the var from Svar
  {
    string str_var = envStr ? envStr : data->get_var(name, "");
    istringstream istr_var(str_var);
    try {
      istr_var >> def;
    } catch (std::exception e) {
      cerr << "Failed to read value from " << str_var << endl;
    }
    while (!ptr) {
      ptr = (typed_map.get_ptr(name, def));
      if (ptr) break;
    }
    return *ptr;
  } else  // Third:Both did not get the var need,insert defaut var to
          // SvarWithType
  {
    while (!ptr) {
      ptr = (typed_map.get_ptr(name, def));
      if (ptr) break;
    }
    if (mode & UPDATE) insert(name, std::to_string(def));
    return *ptr;
  }
}

inline double& Svar::GetDouble(const std::string& name, double def,
                               SVARMODE mode) {
  using namespace std;

  if (!a->d)
    a->d = std::shared_ptr<SvarWithType<double> >(new SvarWithType<double>());

  // First: Use the var from SvarWithType, this is a fast operation
  SvarWithType<double>& typed_map = *(a->d);

  double* ptr = typed_map.get_ptr(name);
  if (ptr) return *ptr;

  bool exist = data->exist(name);
  char* envStr = NULL;
  if (!exist) envStr = getenv(name.c_str());
  if (exist || envStr)  // Second: Use the var from Svar
  {
    string str_var = envStr ? envStr : data->get_var(name, "");
    istringstream istr_var(str_var);
    try {
      istr_var >> def;
    } catch (std::exception e) {
      cerr << "Failed to read value from " << str_var << endl;
    }
    while (!ptr) {
      ptr = (typed_map.get_ptr(name, def));
      if (ptr) break;
    }
    return *ptr;
  } else  // Third:Both did not get the var need,insert defaut var to
          // SvarWithType
  {
    while (!ptr) {
      ptr = typed_map.get_ptr(name, def);
      if (ptr) break;
    }

    if (mode & UPDATE) insert(name, dtos(def, 12));

    return *ptr;
  }
}

inline std::string& Svar::GetString(const std::string& name,
                                    const std::string& def, SVARMODE mode) {
  using namespace std;

  if (!a->s)
    a->s = std::shared_ptr<SvarWithType<string> >(new SvarWithType<string>());
  // First: Use the var from SvarWithType, this is a fast operation
  SvarWithType<string>& typed_map = *(a->s);

  string* ptr = typed_map.get_ptr(name);
  if (ptr) return *ptr;

  if (data->exist(name))  // Second: Use the var from Svar
  {
    string _def = data->get_var(name, def);
    while (!ptr) {
      ptr = (typed_map.get_ptr(name, _def));
      if (ptr) break;
    }
    return *ptr;
  } else  // Third:Both did not get the var need,insert defaut var to
          // SvarWithType
  {
    string _def = def;
    if (char* ptr = getenv(name.c_str())) _def = ptr;
    while (!ptr) {
      ptr = (typed_map.get_ptr(name, _def));
      if (ptr) break;
    }

    if (mode & UPDATE) insert(name, def);

    return *ptr;
  }
}

inline void*& Svar::GetPointer(const std::string& name, const void* ptr,
                               SVARMODE mode) {
  if (!a->p)
    a->p = std::shared_ptr<SvarWithType<void*> >(new SvarWithType<void*>());
  return *(a->p->get_ptr(name, (void*)ptr));
}

inline void Svar::update() {
  using namespace std;
  // from i
  if (a->i) {
    SvarWithType<int>::DataMap data_i = a->i->get_data();
    for (SvarWithType<int>::DataIter it = data_i.begin(); it != data_i.end();
         it++) {
      const string& name = it->first;
      if (!exist(name)) continue;
      insert(name, std::to_string(it->second), true);
    }
  }

  // from d
  if (a->d) {
    SvarWithType<double>::DataMap data_d = a->d->get_data();
    for (SvarWithType<double>::DataIter it = data_d.begin(); it != data_d.end();
         it++) {
      const string& name = it->first;
      if (!exist(name)) continue;
      insert(name, Svar::dtos(it->second, 12));
    }
  }

  // from s
  if (a->s) {
    SvarWithType<string>::DataMap data_s = a->s->get_data();
    for (SvarWithType<string>::DataIter it = data_s.begin(); it != data_s.end();
         it++) {
      const string& name = it->first;
      if (!exist(name)) continue;
      insert(name, it->second);
    }
  }
}

inline bool Svar::save2file(std::string filename) {
  using namespace std;
  if (filename.size() == 0) filename = GetString("Config_File", "Default.cfg");
  ofstream ofs(filename.c_str());
  if (!ofs.is_open()) return false;

  SvarMap data_copy;
  { data_copy = data->get_data(); }
  for (SvarIter it = data_copy.begin(); it != data_copy.end(); it++) {
    ofs << it->first << " = " << it->second << endl;
  }
  return true;
}

inline std::string Svar::getStatsAsText() {
  using namespace std;
  SvarMap data = this->data->get_data();

  int count = data.size();
  if (a->i) count += a->i->get_data().size();
  if (a->d) count += a->d->get_data().size();
  if (a->s) count += a->s->get_data().size();
  if (count == 0) return "";
  ostringstream ost;
  string str;

  ost << "================================== Svar report "
         "===============================\n";
  ost << "NAME                                     VALUE                       "
         "         \n";
  str = a->i ? a->i->getStatsAsText() : "";
  if (str != "")
    ost << "-------------------------------------------------------------------"
           "-----------\n"
        << str;
  str = a->d ? a->d->getStatsAsText() : "";
  if (str != "")
    ost << "-------------------------------------------------------------------"
           "-----------\n"
        << str;
  str = a->s ? a->s->getStatsAsText() : "";
  if (str != "")
    ost << "-------------------------------------------------------------------"
           "-----------\n"
        << str;
  // if(data.size()){
  ost << "---------------------------------------------------------------------"
         "---------\n";
  for (SvarIter it = data.begin(); it != data.end(); it++)
    ost << setw(39) << setiosflags(ios::left) << it->first << "  " << setw(39)
        << setiosflags(ios::left) << it->second << endl;
  //}
  ost << "=============================== End of Svar report "
         "===========================\n\n";

  return ost.str();
}

inline void Svar::dumpAllVars() {
  using namespace std;
  cout << endl << getStatsAsText();
}

inline void buildInHandle(void* ptr, std::string command, std::string sParams) {
  using namespace std;
  if (command == "include" || command == "parse") {
    Svar* svar_ptr = (Svar*)ptr;
    svar_ptr->ParseFile(sParams);
  } else if (command == "echo") {
    cout << sParams << endl;
  } else if (command == "GetString") {
    Svar* svar_ptr = (Svar*)ptr;
    svar_ptr->insert(sParams, svar_ptr->GetString(sParams, ""));
  } else if (command == "GetInt") {
    Svar* svar_ptr = (Svar*)ptr;
    svar_ptr->insert(sParams, std::to_string(svar_ptr->GetInt(sParams, 0)));
  } else if (command == "GetDouble") {
    Svar* svar_ptr = (Svar*)ptr;
    svar_ptr->insert(sParams, Svar::dtos(svar_ptr->GetDouble(sParams, 0), 12));
  }
}

inline void systemFunction(void* ptr, std::string, std::string sParams) {
  Svar* svar_ptr = (Svar*)ptr;
  svar_ptr->GetInt("System.Result") = system(sParams.c_str());
}

inline Scommand::Scommand(Svar& var) : language(new SvarLanguage(*this, var)) {
  RegisterCommand("include", buildInHandle, &var);
  RegisterCommand("parse", buildInHandle, &var);
  RegisterCommand("echo", buildInHandle, &var);
  RegisterCommand("GetVar", buildInHandle, &var);
  RegisterCommand("GetInt", buildInHandle, &var);
  RegisterCommand("GetDouble", buildInHandle, &var);
  RegisterCommand("GetString", buildInHandle, &var);
  RegisterCommand("system", systemFunction, &var);
}

inline void Scommand::RegisterCommand(std::string sCommandName,
                                      CallbackProc callback, void* thisptr) {
  CallbackVector& calls = data[sCommandName];
  calls.push_back(CallbackInfoStruct(callback, thisptr));
}

inline void Scommand::UnRegisterCommand(std::string sCommandName) {
  CallbackVector& calls = data[sCommandName];
  calls.clear();
}

inline void Scommand::UnRegisterCommand(std::string sCommandName,
                                        void* thisptr) {
  CallbackVector& calls = data[sCommandName];
  for (int i = static_cast<int>(calls.size()) - 1; i >= 0; i--)
    if (calls[i].thisptr == thisptr) calls.erase(calls.begin() + i);
}

inline void Scommand::UnRegisterCommand(void* thisptr) {
  using namespace std;
  map<string, CallbackVector>& mmCallBackMap = *(data.get_ptr());
  for (map<string, CallbackVector>::iterator i = mmCallBackMap.begin();
       i != mmCallBackMap.end(); i++)
    UnRegisterCommand(i->first, thisptr);
}

inline bool Scommand::Call(std::string sCommand, std::string sParams) {
  if (!data.exist(sCommand)) {
    // cerr << "Can not find command: " << sCommand << "\n\n";
    return false;
  }

  CallbackVector& calls = data[sCommand];
  for (CallbackVector::iterator it = calls.begin(); it != calls.end(); it++)
    it->cbp(it->thisptr, sCommand, sParams);
  return true;
}

/** split the command and paraments from a string
 * eg:Call("shell ls"); equal Call("shell","ls");
 */
inline bool Scommand::Call(const std::string& sCommand) {
  size_t found = sCommand.find_first_of(" ");
  //    cout<<"sCommand="<<sCommand<<"\nFound="<<found<<"\nCommand="<<sCommand.substr(0,found)<<"\nParaments="<<sCommand.substr(found+1);
  if (found < sCommand.size())
    return Call(sCommand.substr(0, found), sCommand.substr(found + 1));
  else
    return Call(sCommand, "");
}

inline Scommand& Scommand::instance() {
  static thread_local std::shared_ptr<Scommand> g_Scommand(new Scommand());
  return *g_Scommand;
}
}

#endif
