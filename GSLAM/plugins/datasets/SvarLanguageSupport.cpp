#include "Svar.h"

namespace GSLAM {

/// This is for the old Svar version style support
template <typename VarType, typename KeyType=std::string>
class SvarMapHolder{
  friend class Svar;

 public:
  typedef std::map<KeyType, VarType>  DataMap;
  typedef typename DataMap::iterator DataIter;
  typedef std::pair<DataIter, bool>  InsertRet;

 public:
  SvarMapHolder() {}

  /** This gives us singletons instance. \sa enter */
  static SvarMapHolder& instance() {
    static std::shared_ptr<SvarMapHolder> inst(new SvarMapHolder());
    return *inst;
  }

  bool empty()const {return data.empty();}

  inline bool exist(const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    return data.find(name) != data.end();
  }

  inline bool erase(const KeyType& name) {
    std::unique_lock<std::mutex> lock(mMutex);
    data.erase(name);
    return true;
  }

  virtual inline void clear() {
    std::unique_lock<std::mutex> lock(mMutex);
    data.clear();
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

  inline DataMap get_data() {
      std::unique_lock<std::mutex> lock(mMutex);
      return data;
  }

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

  template<typename STREAM>
  void dump(STREAM& stream,const size_t column_width = 80,
            typename std::enable_if<detail::has_saving_support<STREAM, VarType>::value >::type* = 0)
  {
      std::unique_lock<std::mutex> lock(mMutex);

      for (DataIter it = data.begin(); it != data.end(); it++)
          stream<<Svar::printTable({{column_width/2-1,Svar::toString(it->first)},
                            {column_width/2,Svar::toString(it->second)}});
  }

  template<typename STREAM>
  void dump(STREAM& stream,const size_t column_width = 80,
            typename std::enable_if<!detail::has_saving_support<STREAM, VarType>::value >::type* = 0)
  {
      std::unique_lock<std::mutex> lock(mMutex);

      for (DataIter it = data.begin(); it != data.end(); it++)
          stream<<Svar::printTable({{column_width/2-1,Svar::toString(it->first)},
                            {column_width/2,"Unstreamable"}});
  }

  virtual std::string getStatsAsText(const size_t column_width = 80) {
      std::ostringstream ost;
      std::string key_name=Svar::type_id<KeyType>();
      std::string type_name=Svar::type_id<VarType>();
      type_name="map<"+key_name+","+type_name+">";
      int gap=std::max<int>((column_width-type_name.size())/2-1,0);
      for(int i=0;i<gap;i++) ost<<'-';ost<<type_name;
      for(int i=0;i<gap;i++) ost<<'-';ost<<std::endl;
      dump(ost,column_width);
      return ost.str();
  }
 protected:
  DataMap    data;
  std::mutex mMutex;
};

template <typename VarType = void*, typename KeyType = std::string>
using SvarWithType=SvarMapHolder<VarType,KeyType>;

/// This is used to support the old Svar *.cfg configure file
class SvarConfigLanguage {
    typedef std::function<void(std::string sCommand,std::string sParams)> CallbackProc;
    typedef std::vector<CallbackProc> CallbackVector;

 public:
  SvarConfigLanguage(Svar var=Svar()){
      using namespace std::placeholders;
      svar_=Svar::object();
      svar_.call("update",var);
      RegisterCommand("include", [&](std::string sParams){
          svar_.ParseFile(sParams);},_2);
      RegisterCommand("parse", [&](std::string sParams){
          svar_.ParseFile(sParams);},_2);
      RegisterCommand("echo", [&](std::string sParams){
          std::cout << sParams << std::endl;},_2);
      RegisterCommand("GetInt", [&](std::string name,std::string sParams){
          svar_.Set(name,Svar::toString(svar_.Get<int>(sParams)));},_1,_2);
      RegisterCommand("GetDouble", [&](std::string name,std::string sParams){
          svar_.Set(name,Svar::toString(svar_.Get<double>(sParams)));},_1,_2);
      RegisterCommand("system", [&](std::string sParams){
          svar_.Set("System.Result",Svar::toString(system(sParams.c_str())));},_2);

      RegisterCommand(".", &SvarConfigLanguage::collect_line, this, _1, _2);
      RegisterCommand("function", &SvarConfigLanguage::function, this, _1, _2);
      RegisterCommand("endfunction", &SvarConfigLanguage::endfunction, this, _1, _2);
      RegisterCommand("if", &SvarConfigLanguage::gui_if_equal, this, _1, _2);
      RegisterCommand("else", &SvarConfigLanguage::gui_if_else, this, _1, _2);
      RegisterCommand("endif", &SvarConfigLanguage::gui_endif, this, _1, _2);
  }

  template <typename Func,typename... Args>
  inline void RegisterCommand(std::string sCommandName,
                                        Func&& func, Args&& ... args) {
      data.insert(sCommandName,std::bind(std::forward<Func>(func),
                                         std::forward<Args>(args)...));
  }

  inline void UnRegisterCommand(std::string sCommandName) {
      data.erase(sCommandName);
  }

  bool Call(std::string sCommand, std::string sParams){
      if (!data.exist(sCommand)) {
        return false;
      }

      CallbackProc& calls = data[sCommand];
      calls(sCommand,sParams);
      return true;
    }

  /** split the command and paraments from a string
   * eg:Call("shell ls"); equal Call("shell","ls");
   */
  bool Call(const std::string& sCommand){
      size_t found = sCommand.find_first_of(" ");
      if (found < sCommand.size())
        return Call(sCommand.substr(0, found), sCommand.substr(found + 1));
      else
        return Call(sCommand, "");
    }

  virtual void clear(){data.clear();}

  virtual std::string getStatsAsText(const size_t column_width=80){
    return data.getStatsAsText(column_width);
  }

  inline bool ParseLine(std::string s, bool bSilentFailure=false) {
    using namespace std;
    if (s == "") return 0;
    if (collectFlag) {
      istringstream ist(s);
      string sCommand;
      ist >> sCommand;
      if (sCommand == "endif" || sCommand == "fi") Call("endif");
      if (sCommand == "else") Call("else");
      else if (sCommand == "endfunction") Call("endfunction");
      else if (sCommand == ".") Call(".", ist.str());
      else Call(".", s);
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
    if (Call(sCommand, sParams)) return true;

    if (setvar(s)) return 1;

    if (!bSilentFailure)
      cerr << "? GUI_impl::ParseLine: Unknown command \"" << sCommand
           << "\" or invalid assignment." << endl;
    return 0;
  }

  inline bool ParseStream(std::istream& is) {
    using namespace std;
    svar_.Set("Svar.ParsingPath", getFolderPath(parsingFile));
    svar_.Set("Svar.ParsingName", getBaseName(parsingFile));
    svar_.Set("Svar.ParsingFile", parsingFile);
    string buffer;
    int& shouldParse = svar_.GetInt("Svar.NoReturn", 1);
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


  static std::string getFolderPath(const std::string& path) {
    auto idx = std::string::npos;
    if ((idx = path.find_last_of('/')) == std::string::npos)
      idx = path.find_last_of('\\');
    if (idx != std::string::npos)
      return path.substr(0, idx);
    else
      return "";
  }

  static std::string getBaseName(const std::string& path) {
    std::string filename = Svar::getFileName(path);
    auto idx = filename.find_last_of('.');
    if (idx == std::string::npos)
      return filename;
    else
      return filename.substr(0, idx);
  }

  inline bool ParseFile(std::string sFileName) {
    using namespace std;
    ifstream ifs(sFileName.c_str());

    if (!ifs.is_open()) {
      return false;
    }

    std::string  current_tid=svar_.toString(std::this_thread::get_id());
    std::string& parsing_tid=svar_.GetString("Svar.ParsingThreadId");
    assert(current_tid==parsing_tid||parsing_tid.empty());

    fileQueue.push_back(sFileName);
    parsingFile=sFileName;

    bool ret = ParseStream(ifs);
    ifs.close();

    //    cout<<"Finished parsing "<<fileQueue.back();
    fileQueue.pop_back();
    if (fileQueue.size()) {
      parsingFile = fileQueue.back();
      svar_.Set("Svar.ParsingPath", getFolderPath(parsingFile));
      svar_.Set("Svar.ParsingName", Svar::getFileName(parsingFile));
      svar_.Set("Svar.ParsingFile", parsingFile);
    } else {
      svar_.erase("Svar.ParsingName");
      svar_.erase("Svar.ParsingPath");
      svar_.erase("Svar.ParsingFile");
      parsingFile.clear();
      parsing_tid.clear();
    }
    return ret;
  }

  /**
   = overwrite
  ?= don't overwrite
  */
  inline bool setvar(std::string s) {
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

        svar_.Set(var, val, shouldOverwrite);
        return true;
      }
    }

    return false;
  }
  static Svar loadFile(const std::string& file){
      SvarConfigLanguage lang;
      if(!lang.ParseFile(file)) return Svar();
      return lang.svar_;
  }

private:

  inline std::string expandVal(std::string val, char flag) {
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
          if (svar_.exist(inexpand)) {
            oss << svar_.Get<std::string>(inexpand);
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

  static std::string Trim(const std::string& str,
                                const std::string& delimiters) {
    const size_t f = str.find_first_not_of(delimiters);
    return f == std::string::npos
               ? ""
               : str.substr(f, str.find_last_not_of(delimiters) + 1);
  }

  // Find the open brace preceeded by '$'
  static const char* FirstOpenBrace(const char* str, char flag) {
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
  static const char* MatchingEndBrace(const char* str, char flag) {
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

  static std::vector<std::string> ChopAndUnquoteString(std::string s) {
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

  static std::string::size_type FindCloseBrace(const std::string& s,
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

  inline static std::string UncommentString(std::string s) {
    using namespace std;
    int q = 0;

    for (string::size_type n = 0; n < s.size(); n++) {
      if (s[n] == '"') q = !q;

      if (s[n] == '/' && !q) {
        if (n < s.size() - 1 && s[n + 1] == '/') return s.substr(0, n);
      }
    }

    return s;
  }

  void collect_line(std::string name, std::string args){
    (void)name;
    collection.push_back(args);
  }

  void function(std::string name, std::string args){
    using namespace std;

    collectFlag++;
    vector<string> vs = ChopAndUnquoteString(args);
    if (vs.size() != 1) {
      cerr << "Error: " << name << " takes 1 argument: " << name << " name\n";
      return;
    }

    current_function = vs[0];
    collection.clear();
  }

  void endfunction(std::string name, std::string args){
    using namespace std::placeholders;
    using namespace std;
    collectFlag--;
    if (current_function == "") {
      cerr << "Error: " << name << ": no current function.\n";
      return;
    }

    vector<string> vs = ChopAndUnquoteString(args);
    if (vs.size() != 0) cerr << "Warning: " << name << " takes 0 arguments.\n";

    functions.insert(current_function, collection, true);

    RegisterCommand(current_function, &SvarConfigLanguage::runfunction, this, _1,_2);

    current_function.clear();
    collection.clear();
  }

  void runfunction(std::string name, std::string args){
    (void)args;
    using namespace std;
    vector<string>& v = *functions.get_ptr(name, vector<string>());
    for (unsigned int i = 0; i < v.size(); i++) ParseLine(v[i]);
  }

  void gui_if_equal(std::string name, std::string args){
    (void)name;
    using namespace std;
    string& s = args;
    collectFlag++;
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

  void gui_if_else(std::string name, std::string args){
    (void)name;
    (void)args;
    using namespace std;
    ifbit = collection;
    if (ifbit.empty()) ifbit.push_back("");
    collection.clear();
  }

  void gui_endif(std::string name, std::string args){
    (void)name;
    (void)args;
    using namespace std;
    collectFlag--;
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
      for (unsigned int i = 0; i < ib.size(); i++) ParseLine(ib[i]);
    else
      for (unsigned int i = 0; i < eb.size(); i++) ParseLine(eb[i]);
  }

 protected:
  Svar svar_;
  SvarMapHolder<CallbackProc>   data;
private:
 std::string current_function;
 std::string if_gvar, if_string;
 std::vector<std::string> collection, ifbit, elsebit;
 SvarMapHolder<std::vector<std::string> > functions;

 std::string parsingFile;
 std::vector<std::string> fileQueue;
 int         collectFlag=0;
};

REGISTER_SVAR_MODULE(Cfg){

        SvarClass::Class<SvarConfigLanguage>()
                .def_static("loadFile",&SvarConfigLanguage::loadFile);

        svar["__builtin__"]["Cfg"]=SvarClass::instance<SvarConfigLanguage>();
}

}
