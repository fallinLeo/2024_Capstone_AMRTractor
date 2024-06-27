#ifndef CIRCULARBUTTONH_H
#define CIRCULARBUTTONH_H


#include <QPushButton>

class CircularButton : public QPushButton
{
    Q_OBJECT
public:
    explicit CircularButton(QWidget *parent = nullptr);

signals:

public slots:

protected:
    virtual void paintEvent(QPaintEvent *) override;
    virtual void resizeEvent(QResizeEvent *)override;
};

#endif // CIRCULARBUTTONH_H
