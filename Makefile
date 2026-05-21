# 1. 指定 RISC-V 交叉编译器 (C++ 使用 g++)
CXX = riscv64-unknown-linux-musl-g++

# 2. 指定编译选项 (开启 C++17 支持、O2 级别代码优化、显示所有警告)
CXXFLAGS = -std=c++17 -O2 -Wall

# 3. 定义最终生成的可执行文件名称
TARGET = drone_firmware

# 4. 自动查找当前目录下所有的 .cpp 文件
SRCS = $(wildcard *.cpp)

# 5. 自动将所有的 .cpp 文件名替换为对应的 .o 中间目标文件名
OBJS = $(SRCS:.cpp=.o)

# 6. 默认的终极目标：生成最终的飞控固件
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# 7. 编译规则：告诉系统如何将每一个 .cpp 文件编译成对应的 .o 文件
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# 8. 清理规则：用于一键删除所有编译产生的中间文件和固件
clean:
	rm -f $(OBJS) $(TARGET)