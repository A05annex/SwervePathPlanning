package frc6831.planner;

import net.sourceforge.argparse4j.ArgumentParsers;
import net.sourceforge.argparse4j.inf.ArgumentParser;
import net.sourceforge.argparse4j.inf.ArgumentParserException;
import net.sourceforge.argparse4j.inf.Namespace;
import org.jetbrains.annotations.NotNull;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.awt.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.File;

public class PathPlanner extends Frame implements ActionListener, WindowListener {

    private int m_nSizeX = 565;         // the initial X size of the app window (for images for class website).
    private int m_nSizeY = 574;         // the initial Y size of the app window (for images for class website).
    private final GraphicsConfiguration m_graphicsConfig;       // the graphics configuration of the window device

    private final MenuItem m_menuItemFileLoadField;     // the menu file-load field button
    private final MenuItem m_menuItemFileLoadRobot;     // the menu file-load robot button
    private final MenuItem m_menuItemFileLoadPath;      // the menu file-load path button
    private final MenuItem m_menuItemFileSavePath;      // the menu path save button
    private final MenuItem m_menuItemFileSaveAsPath;    // the menu path save-as button

    private final MenuItem m_menuItemEditClearPath;     // the menu edit - clear the current path and reset to
    // start a new path

    private final MenuItem m_menuItemAnimatePlay;       // play an animation of the current path

    private final PathCanvas m_canvas;                  // the rendering canvas (defined at the end of this file)
    private final Robot m_robot = new Robot();          // the robot description
    private final Field m_field = new Field();          // the field description
    private String m_pathFilename = null;

    public static void main(@NotNull final String[] args) {
        // Setup the commandline argument parser and parse any commandline arguments
        ArgumentParser parser = ArgumentParsers.newFor("PathPlanner").build()
                .description("Swerve Drive Path Planner.");
        parser.addArgument("-r", "--robot")
                .type(String.class)
                .help("a robot description file");
        parser.addArgument("-f", "--field")
                .type(String.class)
                .help("a robot description file");
        String robotDescFile = null;
        String fieldDescFile = null;
        try {
            Namespace parsedArgs = parser.parseArgs(args);
            robotDescFile = parsedArgs.get("robot");
            fieldDescFile = parsedArgs.get("field");
        } catch (ArgumentParserException e) {
            parser.handleError(e);
        }
        // start the path planning window
        try {
            final PathPlanner pathPlanner = new PathPlanner(robotDescFile, fieldDescFile);
            pathPlanner.setVisible(true);
        } catch (final Throwable t) {
            t.printStackTrace();
            System.exit(0);
        }
    }

    private PathPlanner(String robotDescFile, String fieldDescFile) {
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

        final Menu menuFile = createMenu(menubar, "File");
        m_menuItemFileLoadField = createMenuItem(menuFile, "Load Game Field ...", this);
        m_menuItemFileLoadRobot = createMenuItem(menuFile, "Load Robot ...", this);
        menuFile.addSeparator();
        m_menuItemFileLoadPath = createMenuItem(menuFile, "Load Path ...", this);
        m_menuItemFileSavePath = createMenuItem(menuFile, "Save Path", this);
        m_menuItemFileSaveAsPath = createMenuItem(menuFile, "Save Path As ...", this);

        final Menu menuEdit = createMenu(menubar, "Edit");
        m_menuItemEditClearPath = createMenuItem(menuEdit, "Clear Path", this);

        final Menu menuAnimate = createMenu(menubar, "Animate");
        m_menuItemAnimatePlay = createMenuItem(menuAnimate, "Play Path", this);

        // the menubar is configured, now add it
        setMenuBar(menubar);
        // and right now everything is so simple that this is the listener
        addWindowListener(this);

        //------------------------------------------------------------------
        // create a canvas to draw on
        //------------------------------------------------------------------
        if (null != robotDescFile) {
            m_robot.loadRobot(robotDescFile);
        }
        if (null != fieldDescFile) {
            m_field.loadField(fieldDescFile);
        }
        resetTitle();
        m_canvas = new PathCanvas(m_graphicsConfig, m_robot, m_field);
        add(m_canvas, BorderLayout.CENTER);

        //------------------------------------------------------------------
        // Setup the app menu
        //------------------------------------------------------------------
        try {
            Desktop desktop = Desktop.getDesktop();
            desktop.setQuitHandler((e, r) -> exitPathPlaner());
        } catch (UnsupportedOperationException e) {
            System.out.println("No desktop quit handler setup, not supported by this platform.");
        }

    }

    private void resetTitle() {
        setTitle("Swerve Path Planner - " + m_field.getTitle());
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
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Load Field");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading field from: " + file.getAbsolutePath());
            m_field.loadField(file.getAbsolutePath());
            resetTitle();
            m_canvas.repaint();
        } else {
            System.out.println("Load field command cancelled by user.");
        }
    }

    private void loadRobot() {
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Load Robot");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading robot from: " + file.getAbsolutePath());
            m_pathFilename = file.getAbsolutePath();
            m_robot.loadRobot(m_pathFilename);
            m_canvas.resetRobotGeometry();
            m_canvas.repaint();
        } else {
            System.out.println("Load path command cancelled by user.");
        }
    }

    private void loadPath() {
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Load Path");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showOpenDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            System.out.println("Loading path from: " + file.getAbsolutePath());
            m_pathFilename = file.getAbsolutePath();
            m_canvas.loadPath(m_pathFilename);
        } else {
            System.out.println("Load path command cancelled by user.");
        }
    }

    private void savePath() {
        System.out.println("Saving path as: " + m_pathFilename);
        m_canvas.getPath().savePath(m_pathFilename);
    }

    private void savePathAs() {
        JFileChooser fc = new JFileChooser(System.getProperty("user.dir"));
        fc.setDialogTitle("Save Path As");
        fc.setFileFilter(new FileNameExtensionFilter("JSON file", "json"));
        fc.setAcceptAllFileFilterUsed(false);
        if (JFileChooser.APPROVE_OPTION == fc.showSaveDialog(m_canvas)) {
            File file = fc.getSelectedFile();
            m_pathFilename = file.getAbsolutePath();
            if (!m_pathFilename.endsWith(".json")) {
                m_pathFilename += ".json";
            }
            savePath();
        } else {
            System.out.println("Save path command cancelled by user.");
        }

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

        // The 'File' menu
        if (src == m_menuItemFileLoadField) {
            loadGameField();
        } else if (src == m_menuItemFileLoadPath) {
            loadPath();
        } else if (src == m_menuItemFileLoadRobot) {
            loadRobot();
        } else if (src == m_menuItemFileSavePath) {
            if (null == m_pathFilename) {
                savePathAs();
            } else {
                savePath();
            }
        } else if (src == m_menuItemFileSaveAsPath) {
            savePathAs();
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
