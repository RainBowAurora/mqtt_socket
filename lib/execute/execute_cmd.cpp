#include "execute_cmd.h"
#include <cstring>

namespace zros{

int ExecuteCMD(const char *cmd, char *result)
{
    int iRet = -1;
    char buf_ps[CMD_RESULT_BUF_SIZE];
    char ps[CMD_RESULT_BUF_SIZE] = {0};
    FILE *ptr;

    strcpy(ps, cmd);

    if((ptr = popen(ps, "r")) != NULL)
    {
        while(fgets(buf_ps, sizeof(buf_ps), ptr) != NULL)
        {
           strcat(result, buf_ps);
           if(strlen(result) > CMD_RESULT_BUF_SIZE)
           {
               break;
           }
        }
        pclose(ptr);
        ptr = NULL;
        iRet = 0;  // 处理成功
    }
    else
    {
        printf("popen %s error\n", ps);
        iRet = -1; // 处理失败
    }

    return iRet;
}

std::string SystemWithResult(const char *cmd)
{
    char cBuf[CMD_RESULT_BUF_SIZE] = {0};
    std::string sCmdResult;

    ExecuteCMD(cmd, cBuf);
    sCmdResult = std::string(cBuf); // char * 转换为 string 类型
    return sCmdResult;
}

} //end namespace zros