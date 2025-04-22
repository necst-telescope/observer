"use client"

import {
    ReactNode,
    createContext,
    useContext,
    useState,
} from "react"
import styles from './SnackbarProvider.module.scss'
import classNames from "classnames/bind"

type SnackbarContextType = {
    notify: ((level: Severity, message: string) => void) | null
}

const SnackbarContext = createContext<SnackbarContextType>({
    notify: null,
})

export const useSnackbar = () => {
    return useContext(SnackbarContext)
}

export function SnackbarProvider(props: { children: ReactNode }): ReactNode {
    const [show, setShow] = useState(false)
    const [severity, setSeverity] = useState<Severity>("info")
    const [message, setMessage] = useState("")

    const cx = classNames.bind(styles)

    function notify(level: Severity, message: string) {
        setShow(true)
        setSeverity(level)
        setMessage(message)
        setTimeout(() => {
            setShow(false)
        }, 5000)
    }

    return (
        <>
            <SnackbarContext.Provider value={{ notify }}>
                {props.children}
            </SnackbarContext.Provider>
            <div className={cx({ snackbar: true, show })}>
                <div className={cx({
                    severity: true,
                    success: severity == 'success',
                    info: severity == 'info',
                    warning: severity == 'warning',
                    error: severity == 'error',
                })} />
                <div className={styles.message}>
                    {message}
                </div>
            </div>
        </>
    )
}
