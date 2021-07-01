const router = require('express').Router()
const Inventory = require('../model/inventory.model')
const mongoose = require('mongoose')

router.route('/sortByLocation').get(async (req,res) => {
    await Inventory.find({'location':req.query.location})
    .exec((err, inventory) => {
        if (err) return (err)
        res.send(inventory) // return data JSON
    })
})

router.route('/sortByUserId').get(async (req,res) => {
    await Inventory.find({'userId':req.query.userId})
    .exec((err, inventory) => {
        if (err) return err
        res.send(inventory) // return data JSON
    })
})

router.route('/sortByTime').get(async (req,res) => {
    await Inventory.find({'createdAt':
        {
            '$gte' : [
                { "$dateFromString": { "format": "%m-%d-%Y" }},
                ISODate(req.query.time)
              ]
        }
    })
    .exec((err, inventory) => {
        if (err) return err
        res.send(inventory) // return data JSON
    })
})

module.exports = router