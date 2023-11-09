#ifndef QPAINTBOX_H
#define QPAINTBOX_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>

class QPaintBox : public QWidget
{
    Q_OBJECT
public:
    explicit QPaintBox(int aWidth = 200, int aHeigth = 200, QWidget *parent = nullptr);

    void setWidth(int awidth);
    void setHeight(int aheight);
    QPixmap *getCanvas(void);

signals:
    /**
     * @brief OnMousePress
     * Señal que indica cuando un boton del mouse se presiono
     * @param event
     * out: event
     */
    void OnMousePress(QMouseEvent *event);

    /**
     * @brief OnMouseDoubleClick
     * Señal que indica cuando un boton del mouse se presiono 2 veces
     * @param event
     * out: event
     */
    void OnMouseDoubleClick(QMouseEvent *event);

    /**
     * @brief OnMouseRelease
     * Señal que se emite cuando el boton del mouse es liberado
     * @param event
     */
    void OnMouseRelease(QMouseEvent *event);

    /**
     * @brief OnMouseMove
     * Señal que se emite cuando el mouse se mueve por el canvas del painter
     * @param event
     */
    void OnMouseMove(QMouseEvent *event);

    /**
     * @brief OnMouseMove
     * Señal que se emite cuando se mueve la rueda del mouse
     * @param event
     */
    void OnWheelMove(QWheelEvent *event);

public slots:

protected:
    void paintEvent(QPaintEvent *event) Q_DECL_OVERRIDE;
    void resizeEvent(QResizeEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;


private:
    QPixmap *pixelCanvas;

};

#endif // QPAINTBOX_H
