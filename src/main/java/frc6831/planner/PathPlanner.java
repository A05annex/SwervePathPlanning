package frc6831.planner;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import org.jetbrains.annotations.NotNull;

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

    private final MenuItem m_menuItemEditClearPath;    // the menu edit - clear the current path and reset to
                                                        // start a new path

    private final MenuItem m_menuItemAnimatePlay;      // play an animation of the current path


    private final PathCanvas m_canvas;                  // the rendering canvas (defined at the end of this file)

    public static void main(@NotNull final String[] args) {
        Robot robot = new Robot();          // the robot description
        Field field = new Field();          // the field description
        // Setup the commandline argument parser and parse any commandline arguments
        ArgumentParser parser = ArgumentParsers.newFor("PathPlanner").build()
                .description("Swerve Drive Path Planner.");
        parser.addArgument("-r", "--robot")
                .type(String.class)
                .help("a robot description file");
        parser.addArgument("-f", "--field")
                .type(String.class)
                .help("a robot description file");
        try {
            Namespace parsedArgs = parser.parseArgs(args);
            String robotDescFile = parsedArgs.get("robot");
            if (null != robotDescFile) {
                robot.loadRobot(robotDescFile);
            }
            String fieldDescFile = parsedArgs.get("field");
            if (null != fieldDescFile) {
                field.loadField(fieldDescFile);
            }
        } catch (ArgumentParserException e) {
            parser.handleError(e);
        }
        // start the path planning window
        try {
            final PathPlanner pathPlanner = new PathPlanner(robot, field);
            pathPlanner.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    private PathPlanner(Robot robot, Field field) {
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

        final Menu menuFile = createMenu( menubar,"File");
        m_menuItemFileLoadField = createMenuItem(menuFile, "Load Game Field", this);
        m_menuItemFileLoadPath = createMenuItem(menuFile, "Load Path", this);
        m_menuItemFileSavePath = createMenuItem(menuFile, "Save Path", this);

        final Menu menuEdit = createMenu( menubar,"Edit");
        m_menuItemEditClearPath = createMenuItem(menuEdit, "Clear Path", this);

        final Menu menuAnimate = createMenu( menubar,"Animate");
        m_menuItemAnimatePlay = createMenuItem(menuAnimate, "Play Path", this);

        // the menubar is configured, now add it
        setMenuBar(menubar);
        // and right now everything is so simple that this is the listener
        addWindowListener(this);

        //------------------------------------------------------------------
        // create a canvas to draw on
        //------------------------------------------------------------------
        m_canvas = new PathCanvas(m_graphicsConfig, robot, field);
        add(m_canvas, BorderLayout.CENTER);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        Desktop desktop = Desktop.getDesktop();
        desktop.setQuitHandler((e, r) -> exitPathPlaner());

    }

    static private Menu createMenu(MenuBar menubar, String name) {
        final Menu menuNamed = new Menu(name);
        menubar.add(menuNamed);
        return menuNamed;
    }

    static private MenuItem createMenuItem(Menu menu, String name, ActionListener actionListener) {
        MenuItem menuItem = new MenuItem(name);
        menuItem.addActionListener(actionListener);
        menu.add(menuItem);
        return menuItem;
    }

    private void loadGameField() {
    }

    private void loadFPath() {
    }

    private void savePath() {
    }

    private void clearPath() {
        m_canvas.clearPath();
    }

    private void animatePath() {
        m_canvas.animatePath();
    }

    private void exitPathPlaner() {
        // All done, dispose of the frame (window)
        dispose();
    }

    @Override
    public void actionPerformed(ActionEvent event) {
        final Object src = event.getSource();

        // The 'File' manu
        if (src == m_menuItemFileLoadField) {
            loadGameField();
        } else if (src == m_menuItemFileLoadPath) {
            loadFPath();
        } else if (src == m_menuItemFileSavePath) {
            savePath();
        } else if (src == m_menuItemEditClearPath) {
            clearPath();
        } else if (src == m_menuItemAnimatePlay) {
            animatePath();
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
