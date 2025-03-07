import QtQuick 2.9
import QtQuick.Window 2.2
import QtQuick.Controls 2.2
import QtQuick.Controls.Material 2.2

ApplicationWindow {
    id: applicationWindow
    width: 1280
    height: 720
    visible: true
    title: qsTr("AGV Panel")
    visibility: "FullScreen"

    Material.theme: Material.Dark

    Button {
        id: confirm_button
        x: 651
        y: 448
        width: 558
        height: 110
        text: qsTr("Confirm")
        font.pointSize: 50
        autoExclusive: false

        onPressAndHold: {
            mediator.confirm()
        }
    }

    Button {
        id: cancel_button
        x: 651
        y: 578
        width: 558
        height: 110
        text: qsTr("Cancel")
        font.pointSize: 50

        onPressAndHold: {
            mediator.cancel()
        }
    }

    Image {
        id: logo_image
        width: 500
        height: 500

        anchors.verticalCenter: parent.verticalCenter
        anchors.verticalCenterOffset: 0
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: -307
        source: "../image/logo.svg"
        sourceSize.width: 500
        sourceSize.height: 500

        MouseArea {
            anchors.fill: parent

            onPressAndHold: {
                mediator.init_pose()
            }
        }
    }

    Pane {
        id: pane
        x: 651
        y: 36
        width: 558
        height: 384

        Material.elevation: 6

        MouseArea {
            id: exit_mousearea
            anchors.fill: parent

            onPressAndHold: {
                Qt.quit()
            }
        }

        Column {
            id: column
            anchors.fill: parent
            spacing: 60

            Text {
                id: id_text
                text: qsTr("Truck 02")
                anchors.left: parent.left
                anchors.right: parent.right
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                wrapMode: Text.WordWrap
                font.bold: true
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                font.pointSize: 50
                color: Material.color(Material.Grey)
            }

            Text {
                id: target_text
                text: mediator.target
                elide: Text.ElideRight
                horizontalAlignment: Text.AlignHCenter
                verticalAlignment: Text.AlignVCenter
                anchors.left: parent.left
                anchors.right: parent.right
                wrapMode: Text.NoWrap
                font.pointSize: 75
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                color: Material.color(Material.primaryColor)
            }

            Text {
                id: log_text
                height: 50
                text: mediator.result
                elide: Text.ElideNone
                anchors.left: parent.left
                anchors.right: parent.right
                verticalAlignment: Text.AlignBottom
                wrapMode: Text.WordWrap
                anchors.rightMargin: 0
                anchors.leftMargin: 0
                font.pointSize: 15
                color: Material.color(Material.Grey)
            }
        }
    }
}

/*##^##
Designer {
    D{i:0;formeditorZoom:0.6600000262260437;height:720;width:1280}
}
##^##*/
