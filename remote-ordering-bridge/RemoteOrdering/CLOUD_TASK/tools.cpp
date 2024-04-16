#include"tools.h"


Tools::Tools(){

}
Tools::~Tools(){

}

//将json写入文件函数
/*** Add cloud platform logging function--no2 start ***/
void Tools::writeJsonDataToFile(const QByteArray &jsonData, const QString &filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append))
    {
        qDebug() << "Failed to open file for writing:" << file.errorString();
        return;
    }

    qint64 maxSizeInBytes = 50 * 1024 * 1024; // 50MB
    clearFileIfExceedSize(filePath, maxSizeInBytes);


    QTextStream out(&file);
    out << jsonData << "\n";
    out << QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss") << "\n\n";
    file.close();
}

//自动清理文件函数
void Tools:: clearFileIfExceedSize(const QString& filePath, qint64 maxSizeInBytes)
{
    QFileInfo fileInfo(filePath);
    if (!fileInfo.exists()) {
        qDebug() << "File does not exist:" << filePath;
        return;
    }

    qint64 fileSize = fileInfo.size();
    if (fileSize > maxSizeInBytes) {
        QFile file(filePath);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            qDebug() << "Failed to open file for writing:" << file.errorString();
            return;
        }
        file.close();
        qDebug() << QString("File %1 cleared due to exceeding size: %2 bytes").arg(filePath).arg(fileSize);
    }
}

/*** Add cloud platform logging function--no2 end ***/






//数据清洗函数
bool Tools:: checkParenthesis(const char* datac) {
    std::stack<char> st;                                    // Define a stack of character types

    for (int i = 0; i < strlen(datac); ++i) {
        if (datac[i] == '(' || datac[i] == '[' || datac[i] == '{') {
            st.push(datac[i]);
        } else if (datac[i] == ')' || datac[i] == ']' || datac[i] == '}') {
            if (st.empty()) return false;                   // The stack is empty, indicating that there is no corresponding left parenthesis before it

            char ch = st.top();                             // Get the top element of the stack
            st.pop();                                       // Delete stack top element

            if ((datac[i] == ')' && ch != '(') ||
                (datac[i] == ']' && ch != '[') ||
                (datac[i] == '}' && ch != '{')) {
                return false;                               // Mismatched parentheses, returned false
            }
        }
    }

    return st.empty();                                      /*** Determine whether the stack is empty.
                                                             If it is not empty, it indicates that the left parenthesis
                                                             has not been fully matched and returns false ***/
}

char* Tools:: chopLastChar(char* datac) {
    QString str = QString::fromUtf8(datac);                 // Convert char * to QString

        if (!str.isEmpty()) {
            str.chop(1);                                    // Delete last character
        }

        QByteArray bytes = str.toUtf8();                    // Convert QString to QByteArray
        char* result = new char[bytes.size() + 1];          // Create a new string

                                                            // Copy the data from QByteArray to a new string
        memcpy(result, bytes.constData(), bytes.size());
        result[bytes.size()] = '\0';                        // Ensure that the string ends with '\0'

        return result;
}
