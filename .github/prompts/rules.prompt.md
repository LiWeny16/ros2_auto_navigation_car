# AI 编程规范（Markdown版本）(必读)

本规范定义了AI工具（如 GitHub Copilot、ChatGPT）在生成代码时必须遵循的规则，确保代码专业、合理、模块化、可维护，并降低幻觉（不准确输出）的风险。

---

## 总体要求

* **架构明确**：代码应符合企业级专业标准，模块明确解耦。
* **解耦性强**：代码分模块，每个模块负责一部分功能，维护性要高
* **可编译运行**：代码需确保能在标准环境下顺利编译并运行。
* **版本无冲突**：依赖管理使用语义化版本（SemVer），避免版本冲突。

---

## 1. 架构设计与模块划分

* 使用**Clean Architecture**模式，严格划分为：

  * `Entities` 实体层：业务实体定义。
  * `UseCases` 应用用例层：纯业务逻辑实现。
  * `InterfaceAdapters` 接口适配层：适配外部接口。
  * `Frameworks & Drivers` 基础设施层：外部框架与驱动。

* **模块解耦**：

  * 禁止跨层直接调用，只能通过清晰定义的接口进行交互。
  * 各模块接口采用纯抽象接口（interface class）定义。

---

## 2. 代码质量与可读性

* **命名规则**：

  * 类名使用驼峰命名法：`PaymentProcessor`
  * 变量名和函数名使用小写下划线：`process_payment()`
  * 常量全大写加下划线：`MAX_RETRY_COUNT`

* **代码风格**：

  * 严格遵守 Google C++ 风格指南（或企业指定风格）。
  * 每个函数单一职责，长度不超过50行，推荐20行以内。
  * 避免深层次嵌套，最多不超过3层。

---

## 3. 依赖管理与版本控制

* **语义化版本控制（SemVer 2.0.0）**：

  * 所有依赖明确标注版本号，避免使用模糊版本描述。
  * 版本冲突必须主动解决，首选命名空间隔离方式。

* **自动依赖更新**：

  * 推荐配置 `Dependabot` 自动检测依赖安全更新，每周生成更新建议。

* **安全依赖扫描**：

  * 项目 CI 必须包含 OWASP Dependency-Check 或等效工具，确保依赖无已知安全漏洞。

---

## 4. CMake 编写规范（针对C++项目）

* **基础规范**：

```cmake
cmake_minimum_required(VERSION 3.25)
project(MyProject VERSION 1.2.3 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
```

* **模块定义示例**：

```cmake
add_library(payment_processor STATIC)
target_sources(payment_processor PRIVATE src/payment_processor.cpp)
target_include_directories(payment_processor PUBLIC include)
target_link_libraries(payment_processor PRIVATE db_client INTERFACE logger)
```

* **禁止**：

  * 使用全局 `include_directories` 或 `link_directories`。

---

## 5. 测试规范

* 必须编写单元测试，目标覆盖率不低于80%。
* 集成测试用于公共接口，验证跨模块集成是否正常。

示例：

```cmake
enable_testing()
add_executable(payment_processor_tests tests/payment_processor_tests.cpp)
target_link_libraries(payment_processor_tests PRIVATE payment_processor gtest_main)
add_test(NAME PaymentProcessorTests COMMAND payment_processor_tests)
```

---

## 6. AI输出控制与防幻觉规范

AI输出代码时必须：

* **提供出处或参考**：
  生成代码中涉及具体版本或关键技术实现时，应尽可能给出来源引用或官方文档链接。

* **检索增强（RAG）**：
  涉及不确定或新的技术方案时，应主动检索最新的官方文档或权威资料，确保答案的准确性。

---

## 7. 示例Prompt模板（可供AI参考使用）

**（以下模板可复制作为GitHub Copilot/ChatGPT提示模板）**

```text
你是企业级C++架构师，严格遵守《AI编程规范》。

# 任务
实现{具体功能描述}。

# 要求
- 使用Clean Architecture分层架构
- 模块接口明确解耦
- 使用Modern CMake规范编写构建脚本
- 所有依赖明确版本，语义化版本管理
- 代码编译运行无冲突
- 降低AI幻觉风险，如不确定请提供参考链接或标注需人工审阅
```

---

## 附录：推荐工具与参考资料

* 语义化版本规范（SemVer）：[https://semver.org/](https://semver.org/)
* Google C++风格指南：[https://google.github.io/styleguide/cppguide.html](https://google.github.io/styleguide/cppguide.html)
* Clean Architecture 官方参考：[https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html](https://blog.cleancoder.com/uncle-bob/2012/08/13/the-clean-architecture.html)
* OWASP Dependency-Check：[https://owasp.org/www-project-dependency-check/](https://owasp.org/www-project-dependency-check/)
* Dependabot 自动依赖更新：[https://docs.github.com/en/code-security/dependabot](https://docs.github.com/en/code-security/dependabot)

---

## 结语

AI工具需严格遵守以上规范生成代码，任何未明确情况需主动请求人工审阅或额外检索，以确保生成代码质量、可维护性和可靠性。
