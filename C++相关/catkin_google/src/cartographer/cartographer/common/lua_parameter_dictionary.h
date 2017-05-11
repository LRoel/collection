/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
#define CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "cartographer/common/lua.h"
#include "cartographer/common/port.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

// Resolves file paths and file content for the Lua 'read' and 'include'
// functions. Use this to configure where those functions load other files from.
/**
* @brief 解析Lua'read'和'include'功能的文件路径和文件内容.使用它来配置这些功能从哪里加载其他文件.
*/
class FileResolver {
 public:
  virtual ~FileResolver() {}
  virtual string GetFullPathOrDie(const string& basename) = 0;
  virtual string GetFileContentOrDie(const string& basename) = 0;
};

// A parameter dictionary that gets loaded from Lua code.
/**
* @brief 从Lua代码加载的参数字典.
*/
class LuaParameterDictionary {
 public:
  // Constructs the dictionary from a Lua Table specification.
  /**
  * @brief 从Lua Table规范构建字典.
  */
  LuaParameterDictionary(const string& code,
                         std::unique_ptr<FileResolver> file_resolver);

  LuaParameterDictionary(const LuaParameterDictionary&) = delete;
  LuaParameterDictionary& operator=(const LuaParameterDictionary&) = delete;

  // Constructs a LuaParameterDictionary without reference counting.
  /**
  * @brief 构造一个LuaParameterDictionary而不引用计数.
  */
  static std::unique_ptr<LuaParameterDictionary> NonReferenceCounted(
      const string& code, std::unique_ptr<FileResolver> file_resolver);

  ~LuaParameterDictionary();

  // Returns all available keys.
  /**
  * @brief 返回所有可用的键.
  */
  std::vector<string> GetKeys() const;

  // Returns true if the key is in this dictionary.
  /**
  * @brief 如果密钥在此字典中，则返回true.
  */
  bool HasKey(const string& key) const;

  // These methods CHECK() that the 'key' exists.
  /**
  * @brief 这些方法CHECK（）表示'key'存在.
  */
  string GetString(const string& key);
  double GetDouble(const string& key);
  int GetInt(const string& key);
  bool GetBool(const string& key);
  std::unique_ptr<LuaParameterDictionary> GetDictionary(const string& key);

  // Gets an int from the dictionary and CHECK()s that it is non-negative.
  /**
  * @brief 从字典和CHECK（）获取一个int，它是非负的.
  */
  int GetNonNegativeInt(const string& key);

  // Returns a string representation for this LuaParameterDictionary.
  /**
  * @brief 返回此LuaParameterDictionary的字符串表示形式.
  */
  string ToString() const;

  // Returns the values of the keys '1', '2', '3' as the given types.
  /**
  * @brief 返回键“1”，“2”，“3”的值作为给定类型.
  */
  std::vector<double> GetArrayValuesAsDoubles();
  std::vector<string> GetArrayValuesAsStrings();
  std::vector<std::unique_ptr<LuaParameterDictionary>>
  GetArrayValuesAsDictionaries();

 private:
  enum class ReferenceCount { YES, NO };
  LuaParameterDictionary(const string& code, ReferenceCount reference_count,
                         std::unique_ptr<FileResolver> file_resolver);

  // For GetDictionary().
  /**
  * @brief 对于GetDictionary（）.
  */
  LuaParameterDictionary(lua_State* L, ReferenceCount reference_count,
                         std::shared_ptr<FileResolver> file_resolver);

  // Function that recurses to keep track of indent for ToString().
  /**
  * @brief 递归跟踪ToString（）缩进的函数.
  */
  string DoToString(const string& indent) const;

  // Pop the top of the stack and CHECKs that the type is correct.
  /**
  * @brief 弹出堆栈的顶部，并检查类型是否正确.
  */
  double PopDouble() const;
  int PopInt() const;
  bool PopBool() const;

  // Pop the top of the stack and CHECKs that it is a string. The returned value
  // is either quoted to be suitable to be read back by a Lua interpretor or
  // not.
  /**
  * @brief 弹出堆栈的顶部，并检查它是一个字符串.引用的值被引用为适合由Lua解释器读取.
  */
  enum class Quoted { YES, NO };
  string PopString(Quoted quoted) const;

  // Creates a LuaParameterDictionary from the Lua table at the top of the
  // stack, either with or without reference counting.
  /**
  * @brief 从堆栈顶部的Lua表创建LuaParameterDictionary，无论是否引用计数.
  */
  std::unique_ptr<LuaParameterDictionary> PopDictionary(
      ReferenceCount reference_count) const;

  // CHECK() that 'key' is in the dictionary.
  /**
  * @brief CHECK（）中的“键”在字典中.
  */
  void CheckHasKey(const string& key) const;

  // CHECK() that 'key' is in this dictionary and reference it as being used.
  /**
  * @brief CHECK（）中的'key'是在这个字典中引用它被使用.
  */
  void CheckHasKeyAndReference(const string& key);

  // If desired, this can be called in the destructor of a derived class. It
  // will CHECK() that all keys defined in the configuration have been used
  // exactly once and resets the reference counter.
  /**
  * @brief 如果需要，这可以在派生类的析构函数中调用.它将检查（）配置中定义的所有键都已使用一次，并重置参考计数器.
  */
  void CheckAllKeysWereUsedExactlyOnceAndReset();

  // Reads a file into a Lua string.
  /**
  * @brief 将文件读入Lua字符串.
  */
  static int LuaRead(lua_State* L);

  // Handles inclusion of other Lua files and prevents double inclusion.
  /**
  * @brief 处理包含其他Lua文件并防止双重包含.
  */
  static int LuaInclude(lua_State* L);

  lua_State* L_;  // The name is by convention in the Lua World.
  int index_into_reference_table_;

  // This is shared with all the sub dictionaries.
  /**
  * @brief 这与所有的子词典共享.
  */
  const std::shared_ptr<FileResolver> file_resolver_;

  // If true will check that all keys were used on destruction.
  /**
  * @brief 如果为true，将检查所有的密钥是否在销毁时使用.
  */
  const ReferenceCount reference_count_;

  // This is modified with every call to Get* in order to verify that all
  // parameters are read exactly once.
  /**
  * @brief 每次调用Get *时，都会修改这个值，以便验证所有参数是否只读一次.
  */
  std::map<string, int> reference_counts_;

  // List of all included files in order of inclusion. Used to prevent double
  // inclusion.
  /**
  * @brief 包含的所有文件的列表.用于防止双重包容.
  */
  std::vector<string> included_files_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_LUA_PARAMETER_DICTIONARY_H_
