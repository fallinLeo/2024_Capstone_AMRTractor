#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QMessageBox>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)

{
    QMessageBox msg;

    msg.setText("Start Tractor-Trailer!");
    msg.exec();

    ui->setupUi(this);

}


MainWindow::~MainWindow()
{
    delete ui;
}



//start button
void MainWindow::on_pushButton_clicked()
{

}
//stop button
void MainWindow::on_pushButton_2_clicked()
{

}


