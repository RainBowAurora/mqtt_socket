#ifndef __EXECUTE_CMD_H__
#define __EXECUTE_CMD_H__

#include<iostream>

const size_t CMD_RESULT_BUF_SIZE  = 2048;

namespace zros{

int ExecuteCMD(const char *cmd, char *result);
std::string SystemWithResult(const char *cmd);


}//end namespace zros


#endif /*__EXECUTE_CMD_H__*/
