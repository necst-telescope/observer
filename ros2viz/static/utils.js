"strict"

function toMap(obj) {
    return new Map(Object.entries(obj))
}

class DefaultMap extends Map {
    get(key) {
        if (!this.has(key)) {
            this.set(key, this.defaultFunc())
        }
        return super.get(key)
    }

    constructor(defaultFunc, entries) {
        super(entries)
        this.defaultFunc = defaultFunc
    }
}


export { toMap, DefaultMap }
