#ifndef DEBUG_H
#define DEBUG_H

#include <QDialog>

namespace Ui {
class Debug;
}

class Debug : public QDialog
{
    Q_OBJECT

public:

    //Utilizado par enviar este tipo de dato,en caso de querer enviar algun otro dato tengo que crear otra funcion que recibe otro tipo de dato como parametro
    void showUnProcessed(QString info); //datos sin estar procesados

    void showProcessed(QString info); //datos procesados

    explicit Debug(QWidget *parent = nullptr);
    ~Debug();

private:
    Ui::Debug *ui;
};

#endif // DEBUG_H
