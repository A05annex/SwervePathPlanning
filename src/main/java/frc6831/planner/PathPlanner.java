package frc6831.planner;

import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;

public class PathPlanner extends Frame implements ActionListener, WindowListener {

    private int m_nSizeX = 565;         // the initial X size of the app window (for images for class website).
    private int m_nSizeY = 574;         // the initial Y size of the app window (for images for class website).
    private final GraphicsConfiguration m_graphicsConfig;       // the graphics configuration of the window devide

    private final MenuItem m_menuItemFileLoadField;     // the menu file-load field button
    private final MenuItem m_menuItemFileLoadPath;      // the menu file-load path button
    private final MenuItem m_menuItemFileSavePath;      // the menu file-load button

    private final PathCanvas m_canvas;                  // the rendering canvas (defined at the end of this file)

    public static void main(@NotNull final String[] args) {
        // start the path planning window
        try {
            final PathPlanner pathPlanner = new PathPlanner();
            pathPlanner.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    private PathPlanner() {
        //------------------------------------------------------------------
        // setup the window for drawing the field and paths
        //------------------------------------------------------------------
        // resize the default if it doesn't fit in the full screen
        m_graphicsConfig = getGraphicsConfiguration();
        final Rectangle boundsRect = m_graphicsConfig.getBounds();
        if (boundsRect.width < m_nSizeX) m_nSizeX = boundsRect.width;
        if (boundsRect.height < m_nSizeY) m_nSizeY = boundsRect.height;
        setSize(m_nSizeX, m_nSizeY);

        // ----------------------------------------------------------------------------------
        // Create the menu with exit
        // ----------------------------------------------------------------------------------
        final MenuBar menubar = new MenuBar();
        final Menu menuFile = new Menu("File");
        menubar.add(menuFile);
        m_menuItemFileLoadField = new MenuItem("Load Game Field");
        m_menuItemFileLoadField.addActionListener(this);
        menuFile.add(m_menuItemFileLoadField);
        m_menuItemFileLoadPath = new MenuItem("Load Path");
        m_menuItemFileLoadPath.addActionListener(this);
        menuFile.add(m_menuItemFileLoadPath);
        m_menuItemFileSavePath = new MenuItem("Save Path");
        m_menuItemFileSavePath.addActionListener(this);
        menuFile.add(m_menuItemFileSavePath);
        // the menubar is configured, now add it
        setMenuBar(menubar);
        // and right now everything is so simple that this is the listener
        addWindowListener(this);

        //------------------------------------------------------------------
        // create a canvas to draw on
        //------------------------------------------------------------------
        m_canvas = new PathCanvas(m_graphicsConfig);
        add(m_canvas, BorderLayout.CENTER);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        Desktop desktop = Desktop.getDesktop();
        desktop.setQuitHandler((e, r) -> {
                    exitPathPlaner();
                }
        );

    }

    private void loadGameField() {
    }

    private void loadFPath() {
    }

    private void savePath() {
    }

    private void exitPathPlaner() {
        // All done, dispose of the frame (window)
        dispose();
    }

    @Override
    public void actionPerformed(ActionEvent event) {
        final Object src = event.getSource();

        if (src == m_menuItemFileLoadField) {
            loadGameField();
        } else if (src == m_menuItemFileLoadPath) {
            loadFPath();
        } else if (src == m_menuItemFileSavePath) {
            savePath();
        }
    }

    @Override
    public void windowOpened(WindowEvent e) {

    }

    @Override
    public void windowClosing(WindowEvent e) {
        exitPathPlaner();
    }

    @Override
    public void windowClosed(WindowEvent e) {

    }

    @Override
    public void windowIconified(WindowEvent e) {

    }

    @Override
    public void windowDeiconified(WindowEvent e) {

    }

    @Override
    public void windowActivated(WindowEvent e) {

    }

    @Override
    public void windowDeactivated(WindowEvent e) {

    }
}
