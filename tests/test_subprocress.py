import subprocess

# 定义要执行的脚本命令
command = "ls -l"  # 以ls -l命令为例，你可以换成你要执行的任何其他脚本命令

# 执行脚本命令
try:
    # 使用subprocess.run()来执行命令
    # capture_output=True 表示捕获标准输出和标准错误
    result = subprocess.run(command, shell=True, text=True, capture_output=True)

    # 输出命令执行结果
    print("标准输出：")
    print(result.stdout)

    print("\n标准错误：")
    print(result.stderr)

    print("\n返回代码：", result.returncode)

except subprocess.CalledProcessError as e:
    print("命令执行出错:", e)